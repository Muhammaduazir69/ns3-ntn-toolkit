#!/usr/bin/env python3
"""ns-3.48 port helper: every Object subclass must define its own static GetTypeId() (ns-3.45+
aborts in ConstructSelf otherwise). Adds an inline GetTypeId() chained to the declared base, right
after the class's first `public:`. Usage: add_typeid.py <group> <file> <Class>..."""
import re, sys
group, path, classes = sys.argv[1], sys.argv[2], sys.argv[3:]
s = open(path).read()
for c in classes:
    m = re.search(r'class\s+' + c + r'\s*(?:final\s*)?:\s*public\s+((?:\w+::)*\w+)\s*\{\s*\n([ \t]*)public:\s*\n', s)
    if not m:
        sys.exit(f'{path}: no "class {c} : public Base {{ public:" layout')
    base, ind = m.group(1), m.group(2)
    # whole class body by brace matching: skip a class that already declares GetTypeId
    i, depth = s.index('{', m.start()), 0
    for j in range(i, len(s)):
        depth += s[j] == '{'; depth -= s[j] == '}'
        if depth == 0:
            break
    if 'GetTypeId' in s[i:j]:
        print(f'{path}: {c} already has GetTypeId, skipped'); continue
    body = (f'{ind}    /**\n{ind}     * @brief Get the type ID (ns-3.48: every Object subclass needs its own).\n'
            f'{ind}     * @return the object TypeId\n{ind}     */\n'
            f'{ind}    static TypeId GetTypeId()\n{ind}    {{\n'
            f'{ind}        static TypeId tid = TypeId("ns3::{c}").SetParent<{base}>().SetGroupName("{group}");\n'
            f'{ind}        return tid;\n{ind}    }}\n\n')
    s = s[:m.end()] + body + s[m.end():]
    print(f'{path}: {c} -> parent {base}')
open(path, 'w').write(s)
