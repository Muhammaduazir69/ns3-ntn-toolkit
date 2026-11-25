#!/usr/bin/env python3

# Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
#
# SPDX-License-Identifier: GPL-2.0-only

"""MR review: run a review agent, driven by a self-hosted LLM, over a merge request.

The agent is Claude Code, following the rules in review-rules.md. The model behind it is
reached through an OpenAI-compatible API (--llm-url); a LiteLLM proxy started by this
script translates between the two.

The context window of the model is small, so the review runs in several short agent
sessions, one per part of a commit, and a last one judges the merge request as a whole.

Output: the verdict and the reason for it in the job log; the full review in review.md
(what each commit does, the verdict, the findings and how to fix each one). The exit code is 1 if the MR is rejected.

Stdlib only; needs git, and the pip packages claude-agent-sdk (it carries the Claude Code
program) and litellm[proxy].
"""

import argparse
import concurrent.futures
import json
import os
import re
import secrets
import shutil
import socket
import subprocess
import sys
import tempfile
import textwrap
import time
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
# Size of the part of a commit given to one agent session, in characters of diff
PART_CHARS = 16000
# Limits of one agent session
SESSION_TURNS = 40
SESSION_SECONDS = 1800
SESSION_ATTEMPTS = 3
# Longest answer of the model in one turn, reasoning included
SESSION_OUTPUT_TOKENS = 16384
# Time given to the LiteLLM proxy to start
PROXY_START_SECONDS = 180
PROXY_POLL_SECONDS = 2
# Context window assumed when the server does not report one
DEFAULT_CTX = 32768
GIT_COMMANDS = ("log", "show", "diff", "grep", "ls-tree", "blame")

# Model names of litellm.yaml by reasoning effort
EFFORTS = {"xhigh": "reviewer", "medium": "reviewer-medium", "low": "reviewer-low"}
EFFORTS["none"] = "reviewer-quick"

# Rules as named in review-rules.md, with their heading in the report, in its order
RULES = {
    "References to earlier code": "Text describing earlier code or earlier versions",
    "Readability": "Hard to read constructs",
    "Undocumented changes": "Changes the commit message does not state",
    "Messages match the code": "Messages, documentation or comments that do not match the code",
    "Duplicate functions": "New functions that duplicate existing ones",
    "Documentation brevity": "Documentation that should be shorter",
    "Notes brevity": "Commit messages and notes that should be shorter",
}
REJECTION_RULE = next(iter(RULES))
VERDICTS = {
    "REJECT": "rejected",
    "SPLIT": "to be split into several merge requests",
    "CHANGES REQUESTED": "changes requested",
    "OK": "good as it is",
    "UNAVAILABLE": "the review could not be completed",
}

PART_REQUEST = """Review one part of the merge request BASE={base} HEAD={head}.

Your part is in the file {path}: {what}. Read that file first.
Check every rule on it except Scope, which someone else checks.

Answer in exactly this form:

## Description
<one to three plain sentences: what this part of the commit actually does, judged from
the diff, not copied from the commit message>

<the findings, in the format of a finding, grouped by rule; or the single line
"No findings." if no rule is broken>"""

VERIFY_REQUEST = """Verify findings on the merge request BASE={base} HEAD={head}.

Another reviewer went quickly over the file {path} ({what}) and listed possible findings
in the file {findings}. That reviewer is often wrong. Read both files, then check every
finding against the rule it is filed under, as that rule is written in your rules. Look
things up with git when the two files are not enough.

A finding is confirmed only when the quoted line is a line the merge request adds (or a
line of a commit message), the rule is really broken by it, and what the finding says
about the code is true. Dismiss a finding that is filed under the wrong rule, that
reports what the rule allows, or that its own text calls borderline or optional.

Answer with one line per finding and nothing else, in exactly this form:

<number>: CONFIRMED
<number>: DISMISSED: <the reason, one sentence>"""

DEDUP_REQUEST = """Remove repeated findings from the review of the merge request
BASE={base} HEAD={head}.

Several reviewers each checked a part of the merge request. The file {path} holds their
findings, numbered. Read it. Some findings say the same thing: two reviewers reported the
same line, or the same problem was reported again for another line or file (the same
wording problem, the same label, the same environment variable).

For every finding that repeats another one, answer with a line in exactly this form,
keeping the clearest of the group as <kept>:

<number>: SAME AS <kept>

Answer with those lines and nothing else. If no finding repeats another, answer with the
single line: No repeats."""

REPORT_REQUEST = """Finish the review of the merge request BASE={base} HEAD={head}.

Other reviewers checked every part of every commit. The file {path} lists the commits,
the files they change, what the reviewers saw each part do, and their findings. Read that
file first. Then:

1. Say what each commit actually does.
2. Check the Scope rule. Use `git show --stat <sha>` or `git show <sha> -- <path>` if
   the file is not enough to tell the topics apart.
3. Tell the author what to do.

Answer in exactly this form, with these headings, the last one only if split:

COMMITS:
<one line per commit, oldest first: its short hash, a colon, then one or two plain
sentences on what it actually does>

SCOPE: keep or split
<the reason, in at most three sentences. If split, then one line per proposed merge
request, in merge order: a title, its commits, and its files only when a commit has to
be divided>

SUMMARY:
<the verdict explained, for the job log, where nothing else of your answer is printed: in
at most six plain sentences, whether the merge request is good as it is, and if not why
not and what the author has to do, most important first. Give the reasons, not the list:
the proposed merge requests and the individual findings are in the full report, which
is not printed with it, so never write "above" or "below". Put a
blank line between the reasons when there are several.>

HOW TO SPLIT:
<only if split: the steps to carry out the split, written for this merge request, with
the git commands of each step in a ``` block. Otherwise leave this heading out.>"""


def git(*args):
    return subprocess.run(["git", *args], check=True, capture_output=True, text=True).stdout


# ------------------------------------------------------------------ parts of the MR


def numbered_files(diff):
    """Per file of a diff: (path, text with the new line number in front of each line)."""
    files = []
    for segment in re.split(r"^(?=diff --git )", diff, flags=re.M):
        if not segment.startswith("diff --git ") or "\nBinary files " in segment:
            continue
        path = re.match(r"diff --git a/.* b/(.*)", segment).group(1)
        lines = [f"=== {path} ==="]
        new_line = 0
        for line in segment.splitlines():
            hunk = re.match(r"@@ -\d+(?:,\d+)? \+(\d+)", line)
            if hunk:
                new_line = int(hunk.group(1))
                lines.append("")
            elif line.startswith(("+++", "---")) or not new_line:
                continue
            elif line.startswith("-"):
                lines.append(f"{'':>6} - {line[1:]}")
            else:
                lines.append(f"{new_line:>6} {line[:1]} {line[1:]}")
                new_line += 1
        files.append((path, "\n".join(lines) + "\n"))
    return files


def cut(files, budget):
    """Group the files of a commit in parts of about budget characters.

    A file larger than the budget is cut at line boundaries.
    """
    pieces = []
    for path, text in files:
        lines = text.splitlines(keepends=True)
        while lines:
            size = 0
            count = 0
            while count < len(lines) and size + len(lines[count]) <= budget:
                size += len(lines[count])
                count += 1
            count = max(count, 1)
            head = "" if lines[0].startswith("===") else f"=== {path} (continued) ===\n"
            pieces.append((path, head + "".join(lines[:count])))
            lines = lines[count:]
    parts = []
    for path, text in pieces:
        if parts and parts[-1]["size"] + len(text) <= budget:
            parts[-1]["paths"].append(path)
            parts[-1]["text"] += "\n" + text
            parts[-1]["size"] += len(text)
        else:
            parts.append({"paths": [path], "text": text, "size": len(text)})
    return parts


def plan(base, head, work):
    """Write one file per part of each commit; returns (commits, parts)."""
    commits = []
    parts = []
    for sha in git("rev-list", "--reverse", "--no-merges", f"{base}..{head}").split():
        short = sha[:8]
        message = git("show", "-s", "--format=%B", sha).strip()
        files = numbered_files(git("show", "--format=", "-U3", "-M", sha))
        paths = [path for path, _ in files]
        commit = {"sha": short, "subject": message.splitlines()[0], "paths": paths}
        commit["descriptions"] = []
        commits.append(commit)
        commit_parts = cut(files, PART_CHARS)
        for number, part in enumerate(commit_parts, 1):
            what = f"commit {short}"
            note = (
                "Cite a finding in the wording of this message as "
                f"`commit-{short}-message:<line of the message>`.\n\n"
            )
            if len(commit_parts) > 1:
                what = f"part {number} of {len(commit_parts)} of commit {short}"
                others = sorted(set(paths) - set(part["paths"]))
                note = (
                    f"This file holds {what}. The rest of the commit is not in this file; "
                    f"it also changes: {', '.join(others) or 'other lines of the same files'}.\n\n"
                )
                if number > 1:
                    note += (
                        "The wording of the commit message is reviewed with part 1: do not "
                        "report it here. Use the message only to judge this part of the diff.\n\n"
                    )
                else:
                    note += (
                        "Cite a finding in the wording of the message as "
                        f"`commit-{short}-message:<line of the message>`.\n\n"
                    )
            path = os.path.join(work, f"part-{len(parts) + 1:03d}.diff")
            with open(path, "w") as file:
                file.write(f"Message of commit {short}:\n\n{message}\n\n{note}")
                file.write("Diff. Each line: line number in the new file, then + or -.\n\n")
                file.write(part["text"])
            paths_of_part = sorted(set(part["paths"]))
            parts.append({"path": path, "what": what, "paths": paths_of_part, "commit": commit})
    return commits, parts


# ------------------------------------------------------------------ agent


def http_json(url, key="", timeout=30):
    request = urllib.request.Request(url, headers={"Authorization": f"Bearer {key}"})
    with urllib.request.urlopen(request, timeout=timeout) as response:
        return json.loads(response.read())


def start_proxy(args, work):
    """Start the LiteLLM proxy; returns (process, environment of the agent)."""
    model = http_json(f"{args.llm_url}/models", args.llm_api_key)["data"][0]
    key = "sk-" + secrets.token_hex(16)
    # The agent reaches its proxy directly, whatever proxy the job has for the rest
    for name in ("NO_PROXY", "no_proxy"):
        os.environ[name] = ",".join(filter(None, [os.environ.get(name), "127.0.0.1"]))
    proxy_env = dict(
        os.environ,
        LITELLM_LOCAL_MODEL_COST_MAP="True",
        REVIEW_LLM_URL=args.llm_url,
        REVIEW_PROXY_MODEL=f"hosted_vllm/{model['id']}",
        REVIEW_PROXY_UPSTREAM_KEY=args.llm_api_key or "none",
        REVIEW_PROXY_KEY=key,
    )
    command = ["litellm", "--config", os.path.join(HERE, "litellm.yaml")]
    with socket.socket() as probe:
        # A port nobody listens on
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
    command += ["--host", "127.0.0.1", "--port", str(port)]
    log = open(os.path.join(work, "litellm.log"), "w")
    proxy = subprocess.Popen(command, env=proxy_env, stdout=log, stderr=subprocess.STDOUT)
    url = f"http://127.0.0.1:{port}"
    start = time.time()
    while time.time() - start < PROXY_START_SECONDS:
        if proxy.poll() is not None:
            raise RuntimeError(f"litellm exited with code {proxy.returncode}")
        try:
            http_json(f"{url}/health/liveliness", key, timeout=PROXY_POLL_SECONDS)
            break
        except OSError:
            time.sleep(PROXY_POLL_SECONDS)
    else:
        proxy.terminate()
        raise RuntimeError("litellm did not start")

    agent_env = dict(
        os.environ,
        ANTHROPIC_BASE_URL=url,
        ANTHROPIC_API_KEY=key,
        ANTHROPIC_MODEL="reviewer",
        ANTHROPIC_DEFAULT_HAIKU_MODEL="reviewer",
        ANTHROPIC_DEFAULT_SONNET_MODEL="reviewer",
        ANTHROPIC_DEFAULT_OPUS_MODEL="reviewer",
        CLAUDE_CODE_MAX_CONTEXT_TOKENS=str(model.get("max_model_len", DEFAULT_CTX)),
        CLAUDE_CODE_MAX_OUTPUT_TOKENS=str(SESSION_OUTPUT_TOKENS),
        CLAUDE_CODE_DISABLE_NONESSENTIAL_TRAFFIC="1",
        CLAUDE_CODE_DISABLE_EXPERIMENTAL_BETAS="1",
        CLAUDE_CONFIG_DIR=os.path.join(work, "claude-config"),
    )
    for name in ("ANTHROPIC_AUTH_TOKEN", "CLAUDE_CODE_OAUTH_TOKEN"):
        agent_env.pop(name, None)
    print(f"review agent on {model['id']} at {args.llm_url}")
    return proxy, agent_env


def claude_program():
    """Path of Claude Code: on PATH, or the copy inside the claude-agent-sdk package."""
    program = shutil.which("claude")
    if not program:
        import claude_agent_sdk

        program = os.path.join(os.path.dirname(claude_agent_sdk.__file__), "_bundled", "claude")
    return program


def run_agent(env, request, name, work, model, form):
    """Run an agent session until it answers in the requested form, three times at most.

    form is a regular expression the answer has to match. Returns (answer or None, note).
    """
    env = dict(env, ANTHROPIC_MODEL=model)
    for tier in ("HAIKU", "SONNET", "OPUS"):
        env[f"ANTHROPIC_DEFAULT_{tier}_MODEL"] = model
    failures = []
    for _ in range(SESSION_ATTEMPTS):
        answer, note = run_session(env, request, name, work)
        # The model sometimes ends a session with a tool call it failed to write properly
        if answer and (not re.search(form, answer, re.M | re.I) or "<function=" in answer):
            answer, note = None, f"answer not in the requested form ({note})"
        if answer is not None:
            return answer, note + (f", after {len(failures)} failed attempts" if failures else "")
        failures.append(note)
    return None, "; ".join(failures)


def run_session(env, request, name, work):
    """Run one agent session; returns (its answer, or None if it failed, and a note)."""
    command = [claude_program(), "-p", "--bare"]
    command += ["--system-prompt-file", os.path.join(HERE, "review-rules.md")]
    command += ["--tools", "Bash,Read", "--allowedTools", "Read"]
    command += [f"Bash(git {git_command}:*)" for git_command in GIT_COMMANDS]
    command += ["--permission-mode", "dontAsk", "--max-turns", str(SESSION_TURNS)]
    command += ["--output-format", "json", request]
    start = time.time()
    try:
        done = subprocess.run(
            command,
            env=env,
            stdin=subprocess.DEVNULL,
            capture_output=True,
            text=True,
            timeout=SESSION_SECONDS,
        )
        output = json.loads(done.stdout)
    except subprocess.TimeoutExpired:
        return None, f"no answer after {SESSION_SECONDS} s"
    except ValueError:
        return None, f"agent failed: {(done.stderr or done.stdout).strip()[-300:]}"
    answer = (output.get("result") or "").strip()
    with open(os.path.join(work, f"{name}.md"), "w") as file:
        file.write(answer + "\n")
    note = f"{output.get('num_turns', '?')} turns, {time.time() - start:.0f} s"
    if output.get("is_error") or not answer:
        return None, f"agent stopped: {answer[:200] or output.get('subtype')} ({note})"
    return answer, note


# ------------------------------------------------------------------ report


def findings_of(answer):
    """Split the answer of a part session into (description, [(rule, finding), ...])."""
    found = []
    description = ""
    for section in re.split(r"^## *", answer, flags=re.M)[1:]:
        title, _, body = section.partition("\n")
        if title.strip().lower().startswith("description"):
            description = " ".join(body.replace("No findings.", "").split())
            continue
        rule = next((r for r in RULES if r.lower() in title.lower()), None)
        # One finding per "path:line" line followed by its quote, problem, suggestion, fix
        for finding in re.split(r"\n(?=\S[^\n]*:\d+[^\n]*\n\s+`)", "\n" + body.strip()):
            if rule and "Problem:" in finding:
                # Its lines are indented: an unindented line after the first one is a
                # closing remark of the session, not part of the finding
                lines = finding.strip().splitlines()
                end = next((i for i, l in enumerate(lines) if i and l[:1].strip()), len(lines))
                found.append((rule, "\n".join(lines[:end]).rstrip()))
    return description, found


def verify(env, args, part, name, findings, work):
    """Have a session check the findings of a part; returns (kept findings, note)."""
    path = os.path.join(work, f"{name}-findings.md")
    with open(path, "w") as file:
        for number, (rule, finding) in enumerate(findings, 1):
            file.write(f"# Finding {number}, filed under the rule: {rule}\n\n{finding}\n\n")
    request = VERIFY_REQUEST.format(base=args.base, head=args.head, findings=path, **part)
    answer, note = run_agent(
        env, request, f"{name}-verified", work, EFFORTS[args.effort], r"CONFIRMED|DISMISSED"
    )
    if answer is None:
        return findings, f"not verified, {note}"
    dismissed = {int(number) for number in re.findall(r"^\W*(\d+)\W+DISMISSED", answer, re.M)}
    kept = [finding for number, finding in enumerate(findings, 1) if number not in dismissed]
    return kept, f"{len(kept)} of {len(findings)} findings confirmed, {note}"


def review_part(env, args, part, name, work):
    """Review a part; returns (description, findings or None if it failed, note)."""
    request = PART_REQUEST.format(base=args.base, head=args.head, **part)
    model = EFFORTS[args.part_effort or args.effort]
    form = r"^## *Description\b[^\0]*(No findings|Problem:)"
    answer, note = run_agent(env, request, name, work, model, form)
    if answer is None:
        return "", None, note
    description, findings = findings_of(answer)
    if not findings or not args.verify:
        return description, findings, f"{len(findings)} findings, {note}"
    findings, verified = verify(env, args, part, name, findings, work)
    return description, findings, f"{verified}; sweep: {note}"


def short_of(finding):
    """A finding without its detailed fix."""
    return re.split(r"\n\s*Fix:", finding, maxsplit=1)[0].rstrip()


def without_repeats(env, args, findings, work):
    """Have a session point out repeated findings; returns the findings without them."""
    if len(findings) < 2:
        return findings
    path = os.path.join(work, "numbered-findings.md")
    with open(path, "w") as file:
        for number, (rule, finding) in enumerate(findings, 1):
            file.write(f"# Finding {number} ({RULES[rule]})\n\n{finding}\n\n")
    request = DEDUP_REQUEST.format(base=args.base, head=args.head, path=path)
    answer, note = run_agent(
        env, request, "repeats", work, EFFORTS[args.effort], r"SAME AS|No repeats"
    )
    print(f"repeated findings: {note}", flush=True)
    answered = {}
    for number, kept in re.findall(r"^\W*(\d+)\W+SAME AS\W*(\d+)", answer or "", re.M | re.I):
        answered[int(number)] = int(kept)
    # For each repeated finding, the finding that stays in its place
    same = {}
    for number, kept in answered.items():
        seen = {number}
        while kept in answered and kept not in seen:  # follow chains, whatever the answer
            seen.add(kept)
            kept = answered[kept]
        if kept not in seen and 1 <= kept <= len(findings) and 1 <= number <= len(findings):
            same[number] = kept
    also = {}
    for number, kept in same.items():
        also.setdefault(kept, []).append(findings[number - 1][1].splitlines()[0].strip())
    kept_findings = []
    for number, (rule, finding) in enumerate(findings, 1):
        if number in same:
            continue
        if number in also:
            head, fix = short_of(finding), finding[len(short_of(finding)) :]
            finding = head + "\n  Also at: " + ", ".join(also[number]) + fix
        kept_findings.append((rule, finding))
    return kept_findings


def as_markdown(finding):
    """A finding of a session laid out for reading: words and code in separate blocks."""
    lines = finding.splitlines()
    body = textwrap.dedent("\n".join(lines[1:]))
    fields = re.match(
        r"\s*`(?P<quote>.*)`\s*\n(?P<rest>.*?)Problem:(?P<problem>.*?)"
        r"(?:Suggestion:(?P<suggestion>.*?))?(?:Also at:(?P<also>[^\n]*)\n?)?(?:Fix:(?P<fix>.*))?$",
        body,
        re.S,
    )
    if not fields:
        return f"### {lines[0].strip()}\n\n{body}"
    out = [f"### {lines[0].strip()}", "", "```", fields["quote"], "```", ""]
    out += ["**Problem.** " + " ".join(fields["problem"].split()), ""]
    if fields["suggestion"]:
        out += ["**Suggestion.** " + " ".join(fields["suggestion"].split()), ""]
    if fields["also"]:
        out += ["**Also at:** " + fields["also"].strip(), ""]
    # The fix: its words, then each code block on its own
    fence = r"(^[ \t]*```.*?^[ \t]*```[ \t]*$)"
    for number, piece in enumerate(re.split(fence, fields["fix"] or "", flags=re.S | re.M)):
        if piece.lstrip().startswith("```"):
            out += [textwrap.dedent(piece).strip(), ""]
        elif piece.strip():
            out += [("" if number else "**How to fix.** ") + " ".join(piece.split()), ""]
    return "\n".join(out).rstrip()


def paragraphs(text):
    """Text with a blank line between its blocks and lines of at most 100 columns."""
    blocks = [" ".join(block.split()) for block in re.split(r"\n\s*\n", text.strip())]
    return "\n\n".join(textwrap.fill(block, 100) for block in blocks if block)


def detailed_report(result, commits):
    """The full report, for the artifact."""
    verdict = f"{result['verdict']} ({VERDICTS[result['verdict']]})"
    lines = [f"# MR review: {verdict}", "", "## What each commit does", ""]
    for commit in commits:
        does = result["commits"].get(commit["sha"]) or " ".join(commit["descriptions"])
        lines += [f"**`{commit['sha']}` {commit['subject']}**", "", does, ""]
    lines += ["## Verdict", "", f"**{verdict}.**", "", result["summary"], ""]
    if result["scope"]:
        # A list needs a blank line before it to be shown as one
        scope = re.sub(r"([^\n])\n(?=(?:[-*]|1\.) )", r"\1\n\n", result["scope"], count=1)
        lines += [f"## Scope: {result['scope_verdict']}", "", scope, ""]
    if result["split_steps"]:
        lines += ["### How to split", "", result["split_steps"], ""]
    for rule, title in RULES.items():
        findings = result["findings"].get(rule)
        if findings:
            note = " (reason for the rejection)" if rule == REJECTION_RULE else ""
            lines += [f"## {title}{note}", ""]
            for finding in findings:
                lines += [as_markdown(finding), ""]
    if result["errors"]:
        lines += ["## Not reviewed", ""] + [f"- {error}" for error in result["errors"]] + [""]
    return "\n".join(lines)


def log_report(result, path):
    """The verdict and the reason for it, for the job log."""
    lines = [f"MR review: {result['verdict']} ({VERDICTS[result['verdict']]})", ""]
    if result["summary"]:
        lines += [paragraphs(result["summary"]), ""]
    counts = [
        f"  {len(result['findings'][rule]):3d}  {title}"
        for rule, title in RULES.items()
        if result["findings"].get(rule)
    ]
    if counts:
        lines += ["Findings:", ""] + counts + [""]
    if result["errors"]:
        lines += [f"{len(result['errors'])} parts of the review failed, see the report.", ""]
    lines += [f"Full review, with how to fix each finding: {path} in the artifacts of this job."]
    return "\n".join(lines)


def review(args, work, result):
    """Run the agent sessions and fill in result; returns the commits."""
    commits, parts = plan(args.base, args.head, work)
    proxy, env = start_proxy(args, work)
    try:
        findings = []
        with concurrent.futures.ThreadPoolExecutor(max_workers=args.parallel) as pool:
            sessions = []
            for number, part in enumerate(parts, 1):
                name = f"part-{number:03d}"
                session = pool.submit(review_part, env, args, part, name, work)
                sessions.append((part, session))
            for number, (part, session) in enumerate(sessions, 1):
                description, found, note = session.result()
                print(f"[{number}/{len(parts)}] {part['what']}: {note}", flush=True)
                if found is None:
                    paths = ", ".join(part["paths"])
                    result["errors"].append(f"{part['what']} ({paths}): {note}")
                    continue
                part["commit"]["descriptions"].append(description)
                findings += found
        findings = without_repeats(env, args, findings, work)
        result["findings"] = {rule: [f for r, f in findings if r == rule] for rule in RULES}

        path = os.path.join(work, "findings.md")
        with open(path, "w") as file:
            file.write("# Commits, oldest first\n\n")
            for commit in commits:
                file.write(f"## {commit['sha']} {commit['subject']}\n\nFiles:\n")
                file.write("".join(f"  {p}\n" for p in commit["paths"]))
                file.write("What the reviewers saw its parts do:\n")
                file.write("".join(f"  - {d}\n" for d in commit["descriptions"] if d) + "\n")
            if args.title:
                file.write(f"# Merge request description\n\n{args.title}\n\n{args.description}\n\n")
            file.write("# Findings of the reviewers\n\n")
            for rule, title in RULES.items():
                if result["findings"][rule]:
                    file.write(f"## {title}\n\n" + "\n\n".join(result["findings"][rule]) + "\n\n")
        request = REPORT_REQUEST.format(base=args.base, head=args.head, path=path)
        answer, note = run_agent(
            env,
            request,
            "report",
            work,
            EFFORTS[args.effort],
            r"COMMITS:[^\0]*SCOPE:\s*(keep|split)",
        )
        print(f"commits, scope and summary: {note}", flush=True)
        if answer is None:
            result["errors"].append(f"commits, scope and summary: {note}")
            return commits
        match = re.search(
            r"COMMITS:(.*?)SCOPE:\s*(keep|split)\b(.*?)(?:SUMMARY:(.*?))?(?:HOW TO SPLIT:(.*))?$",
            answer,
            re.S | re.I,
        )
        if not match:
            result["errors"].append("commits, scope and summary: answer not in the requested form")
            return commits
        for sha, does in re.findall(r"^\W*([0-9a-f]{7,40})\W*:\s*(.+)$", match.group(1), re.M):
            result["commits"][sha[:8]] = does.strip()
        result["scope_verdict"] = match.group(2).lower()
        result["scope"] = match.group(3).strip()
        result["summary"] = (match.group(4) or "").strip()
        result["split_steps"] = (match.group(5) or "").strip()
    finally:
        proxy.terminate()
    return commits


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--base", default=os.getenv("CI_MERGE_REQUEST_DIFF_BASE_SHA"))
    parser.add_argument("--head", default="HEAD")
    parser.add_argument("--title", default=os.getenv("CI_MERGE_REQUEST_TITLE", ""))
    parser.add_argument("--description", default=os.getenv("CI_MERGE_REQUEST_DESCRIPTION", ""))
    parser.add_argument(
        "--llm-url",
        default=os.getenv("REVIEW_LLM_URL"),
        help="base URL of an OpenAI-compatible API, e.g. http://127.0.0.1:8000/v1",
    )
    parser.add_argument("--llm-api-key", default=os.getenv("REVIEW_LLM_API_KEY", ""))
    parser.add_argument("--parallel", type=int, default=8, help="agent sessions at a time")
    parser.add_argument(
        "--effort",
        default="xhigh",
        choices=EFFORTS,
        help="how much the model reasons before it answers",
    )
    parser.add_argument(
        "--part-effort",
        choices=EFFORTS,
        help="the same for the sessions reviewing the parts (default: as --effort)",
    )
    parser.add_argument(
        "--verify",
        action="store_true",
        help="have a second session confirm the findings of each part",
    )
    parser.add_argument("--work", help="keep the session files, in this directory")
    parser.add_argument("--output", default="review.md")
    args = parser.parse_args()
    if not args.base:
        sys.exit("no --base and CI_MERGE_REQUEST_DIFF_BASE_SHA unset")
    if not args.llm_url:
        sys.exit("no --llm-url and REVIEW_LLM_URL unset")
    args.llm_url = args.llm_url.rstrip("/")
    args.head = git("rev-parse", args.head).strip()
    args.base = git("merge-base", args.base, args.head).strip()
    work = os.path.abspath(args.work) if args.work else tempfile.mkdtemp(prefix="mr-review-")
    os.makedirs(work, exist_ok=True)

    result = {
        "verdict": "UNAVAILABLE",
        "summary": "",
        "scope_verdict": "",
        "scope": "",
        "commits": {},
        "split_steps": "",
        "findings": {},
        "errors": [],
    }
    commits = []
    try:
        commits = review(args, work, result)
    except Exception as error:  # never fail the pipeline on agent or model trouble
        result["errors"].append(f"review stopped: {type(error).__name__}: {error}")
    finally:
        if not args.work:
            shutil.rmtree(work, ignore_errors=True)

    count = sum(len(findings) for findings in result["findings"].values())
    if result["findings"].get(REJECTION_RULE):
        result["verdict"] = "REJECT"
    elif result["scope_verdict"] == "split":
        result["verdict"] = "SPLIT"
    elif count:
        result["verdict"] = "CHANGES REQUESTED"
    elif result["scope_verdict"]:
        result["verdict"] = "OK"
    print("=" * 96)
    with open(args.output, "w") as file:
        file.write(detailed_report(result, commits))
    print()
    print(log_report(result, args.output))
    sys.exit(1 if result["verdict"] == "REJECT" else 0)


if __name__ == "__main__":
    main()
