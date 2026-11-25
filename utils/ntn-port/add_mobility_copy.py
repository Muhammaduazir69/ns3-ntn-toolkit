#!/usr/bin/env python3
"""ns-3.48 port helper: MobilityModel::Copy() is pure virtual since ns-3.48. Give every toolkit
mobility model its own Copy() (also models derived from GeocentricConstantPositionMobilityModel,
which would otherwise inherit a Copy() that slices them into a fixed-position model).
Inserted right after the class's `static TypeId GetTypeId();`. Usage: add_mobility_copy.py <dir>..."""
import re, sys, pathlib
CLS = re.compile(r'class\s+(\w+)\s*(?:final\s*)?:\s*public\s+(?:ns3::)?(\w*MobilityModel)\b')
for root in sys.argv[1:]:
    for p in pathlib.Path(root).rglob('*'):
        if p.suffix not in ('.h', '.cc') or '/satellite/' in str(p) or '/mmwave/' in str(p) or '/nr/' in str(p):
            continue
        s = p.read_text(errors='ignore'); o = s
        for m in list(CLS.finditer(s)):
            name = m.group(1)
            body_start = s.find('{', m.end())
            # end of class: matching brace
            depth, i = 0, body_start
            while i < len(s):
                depth += s[i] == '{'; depth -= s[i] == '}'
                if depth == 0: break
                i += 1
            body = s[body_start:i]
            if re.search(r'\bCopy\(\)\s*const', body):
                continue
            g = re.search(r'\n([ \t]*)static\s+(?:ns3::)?TypeId\s+GetTypeId\(\);\n', body)
            if not g:
                print('NO GetTypeId, skipped:', p, name); continue
            ind = g.group(1)
            ins = (f'{ind}/// ns-3.48: copy for MobilityModel::Copy() (spectrum wraparound); keeps the dynamics.\n'
                   f'{ind}Ptr<MobilityModel> Copy() const override\n{ind}{{\n{ind}    return CreateObject<{name}>(*this);\n{ind}}}\n')
            pos = body_start + g.end()
            s = s[:pos] + ins + s[pos:]
        if s != o:
            p.write_text(s); print(p)
