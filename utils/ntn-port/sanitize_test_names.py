#!/usr/bin/env python3
"""ns-3.48 port helper: test names may not contain any of  " / \\ | ?  (TestCase::AddTestCase warns
now and will refuse later). Rewrites the string literal passed to a TestCase/TestSuite constructor
(': TestCase("...")', ': TestSuite("...")', 'TestCase(cond ? "a" : "b")') replacing '/' with '-',
'|' with '-', '?' with '' and '\\' with '-'. Usage: sanitize_test_names.py <dir>..."""
import re, sys, pathlib
BAD = str.maketrans({'/': '-', '|': '-', '?': '', '\\': '-'})
PAT = re.compile(r'(Test(?:Case|Suite)\s*\()((?:[^()]|\([^()]*\))*?)(\))', re.S)
LIT = re.compile(r'"((?:[^"\\]|\\.)*)"')
for root in sys.argv[1:]:
    for p in pathlib.Path(root).rglob('*.cc'):
        if '/test/' not in str(p) and '/tests/' not in str(p):
            continue
        s = p.read_text(errors='ignore'); o = s
        def fix_call(m):
            inner = LIT.sub(lambda l: '"' + l.group(1).translate(BAD) + '"' if any(c in l.group(1) for c in '/|?') else l.group(0), m.group(2))
            return m.group(1) + inner + m.group(3)
        s = PAT.sub(fix_call, s)
        if s != o:
            p.write_text(s); print(p)
