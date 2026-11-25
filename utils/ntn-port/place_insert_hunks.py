#!/usr/bin/env python3
"""ns-3.48 port helper: place pure-insertion hunks from a .rej file.

A hunk qualifies only if it has no '-' lines. Each '+' block is inserted right after its
nearest preceding context line, or, if none, right before the next context line, and only
when that anchor line occurs EXACTLY ONCE in the target file (whitespace-exact). Anything
else is left in a residual .rej for manual porting. Prints every placement with its anchor
so each one can be reviewed. Usage: place_insert_hunks.py <file.rej>"""
import re, sys
rej = sys.argv[1]
target = rej[:-4]
src = open(target).read().split('\n')
text = open(rej).read()
header, *hunks = re.split(r'\n(?=@@ )', text)
residual = []
for h in hunks:
    lines = h.split('\n')[1:]
    if any(l.startswith('-') for l in lines):
        residual.append(h); continue
    ok, plan = True, []
    i = 0
    while i < len(lines):
        if lines[i].startswith('+'):
            j = i
            while j < len(lines) and lines[j].startswith('+'):
                j += 1
            block = [l[1:] for l in lines[i:j]]
            before = next((lines[k][1:] for k in range(i - 1, -1, -1) if lines[k].startswith(' ') and lines[k][1:].strip()), None)
            after = next((lines[k][1:] for k in range(j, len(lines)) if lines[k].startswith(' ') and lines[k][1:].strip()), None)
            if before is not None and src.count(before) == 1:
                plan.append(('after', before, block))
            elif after is not None and src.count(after) == 1:
                plan.append(('before', after, block))
            else:
                ok = False; break
            i = j
        else:
            i += 1
    if not ok:
        residual.append(h); continue
    for kind, anchor, block in plan:
        idx = src.index(anchor)
        pos = idx + 1 if kind == 'after' else idx
        src[pos:pos] = block
        print(f'{target}: {len(block)} lines {kind} [{anchor.strip()[:70]}] at line {pos + 1}')
open(target, 'w').write('\n'.join(src))
if residual:
    open(rej, 'w').write(header + '\n' + '\n'.join(residual) + ('\n' if not residual[-1].endswith('\n') else ''))
    print(f'{rej}: {len(residual)} hunk(s) left for manual port')
else:
    import os; os.remove(rej); print(f'{rej}: all placed')
