#!/usr/bin/env python3
"""ns-3.48 port helper: Object::GetInstanceTypeId() is final since ns-3.45. Remove per-class overrides
whose body only returns GetTypeId(), for classes that derive (directly or through other classes in
the scanned tree) from Object. Header, Tag, Trailer, Chunk and other ObjectBase-only classes keep
theirs. The base of each class is looked up from its own `class X : public B` declaration.
Usage: drop_instance_typeid.py <dir>...   Prints every file it changes."""
import re, sys, pathlib
OBJBASE_ONLY = {'Header', 'Tag', 'Trailer', 'Chunk', 'ObjectBase', 'Buffer'}
files = [p for r in sys.argv[1:] for p in pathlib.Path(r).rglob('*') if p.suffix in ('.h', '.cc')]
bases = {}
for p in files:
    for m in re.finditer(r'class\s+(\w+)\s*(?:final\s*)?:\s*(?:public|protected|private)\s+(?:ns3::)?(?:\w+::)*(\w+)', p.read_text(errors='ignore')):
        bases.setdefault(m.group(1), m.group(2))
def objectbase_only(cls, depth=0):
    b = bases.get(cls)
    if b is None or depth > 20:
        return False            # unknown base (from src/): ns-3 classes overriding it are Object-derived
    if b in OBJBASE_ONLY:
        return True
    return objectbase_only(b, depth + 1)
V = r'\((?:void)?\)'
DEF = re.compile(r'\n(?:ns3::)?TypeId\s*\n?\s*((?:\w+::)*(\w+))::GetInstanceTypeId' + V + r'\s*const\s*\{\s*return\s+(?:\w+::)?GetTypeId\(\);\s*\}\n')
INLINE = re.compile(r'^[ \t]*(?:virtual[ \t]+)?TypeId[ \t]+GetInstanceTypeId' + V + r'[ \t]*const[ \t]*(?:override)?[ \t]*\{[ \t]*return[ \t]+(?:\w+::)?GetTypeId\(\);[ \t]*\}[ \t]*\n', re.M)
DECL = re.compile(r'^[ \t]*(?:virtual[ \t]+)?TypeId[ \t]+GetInstanceTypeId' + V + r'[ \t]*const[ \t]*(?:override)?[ \t]*;[ \t]*\n', re.M)
def owner(s, pos):
    """Innermost class declared before pos (by brace depth)."""
    best = None
    for m in re.finditer(r'class\s+(\w+)\s*(?:final\s*)?:[^{;]*\{', s[:pos]):
        depth = s[m.end():pos].count('{') - s[m.end():pos].count('}')
        if depth >= 0:
            best = m.group(1)
    return best
for p in files:
    s = p.read_text(errors='ignore'); o = s
    s = DEF.sub(lambda m: m.group(0) if objectbase_only(m.group(2)) else '\n', s)
    for rx in (INLINE, DECL):
        out, last = [], 0
        for m in rx.finditer(s):
            cls = owner(s, m.start())
            if cls and not objectbase_only(cls):
                out.append(s[last:m.start()]); last = m.end()
        out.append(s[last:]); s = ''.join(out)
    if s != o:
        p.write_text(s); print(p)
