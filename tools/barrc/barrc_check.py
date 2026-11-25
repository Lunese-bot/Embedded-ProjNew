#!/usr/bin/env python3
"""Lexical BARR-C:2018 checker for app_program/ (a subset of the rules).

Usage:
  barrc_check.py ROOT                  print every finding (TSV)
  barrc_check.py ROOT --allow FILE     print only the findings FILE does not
                                       allow; exit 1 if there are any

Each finding: rule<TAB>file<TAB>line<TAB>note.  Comments and strings are
blanked first, so line numbers stay right.  It is a lexical check, not a
compiler: allowlist.tsv records each deliberate deviation (docs/BARR_C.md)
and each known false positive, with the reason.  New code must pass without
new allowlist lines unless the deviation is agreed and documented."""
import fnmatch, os, re, sys

ROOT = sys.argv[1]
ALLOW = sys.argv[3] if (len(sys.argv) > 3 and sys.argv[2] == '--allow') else None

def strip(src):
    """Return (code, comments) with same length; code has comments+strings blanked."""
    out = []; com = []
    i, n = 0, len(src)
    while i < n:
        c = src[i]
        if src.startswith('/*', i):
            j = src.find('*/', i + 2); j = n if j < 0 else j + 2
            seg = src[i:j]
            out.append(re.sub(r'[^\n]', ' ', seg)); com.append(seg)
            i = j
        elif src.startswith('//', i):
            j = src.find('\n', i); j = n if j < 0 else j
            seg = src[i:j]; out.append(' ' * len(seg)); com.append(seg); i = j
        elif c in '"\'':
            j = i + 1
            while j < n and src[j] != c:
                j += 2 if src[j] == '\\' else 1
            j += 1
            seg = src[i:j]
            out.append(c + re.sub(r'[^\n]', ' ', seg[1:-1]) + c)
            com.append(re.sub(r'[^\n]', ' ', seg)); i = j
        else:
            out.append(c); com.append('\n' if c == '\n' else ' '); i += 1
    return ''.join(out), ''.join(com)

F = []
LOCALS = []
PARAMS = []
GLOBALS_ = []
def add(rule, path, line, note):
    F.append((rule, path, line, note.strip().replace('\t', ' ')[:110]))

def lineno(code, pos):
    return code.count('\n', 0, pos) + 1

def match_paren(code, i):
    """i at '(' ; return index after matching ')'."""
    d = 0
    for j in range(i, len(code)):
        if code[j] == '(':
            d += 1
        elif code[j] == ')':
            d -= 1
            if d == 0:
                return j + 1
    return len(code)

SCALAR = r'(?:unsigned|signed|int|short|long|char|float|double|bool|BOOL|[a-zA-Z_]\w*_t|INT|UINT|UB|UH|UW|B|H|W|VB|VH|VW|ER|ID|PRI|TMO|SZ|FP|SYSTIM|RELTIM)'
KW = set('if else for while do switch case default return break continue goto sizeof typedef struct union enum static const volatile extern register auto inline void'.split())

files = []
for d, _, fs in os.walk(ROOT):
    for f in fs:
        if f.endswith(('.c', '.h')):
            files.append(os.path.join(d, f))
files.sort()

all_headers_text = ''
for p in files:
    if p.endswith('.h'):
        all_headers_text += strip(open(p, encoding='utf-8', errors='replace').read())[0]

for path in files:
    rel = os.path.relpath(path, ROOT)
    raw = open(path, 'rb').read()
    src = raw.decode('utf-8', errors='replace')
    code, comm = strip(src)
    pcode = code
    _ls = code.split('\n'); _cont = False
    for _i, _l in enumerate(_ls):
        if _cont or _l.lstrip().startswith('#'):
            _cont = _l.rstrip().endswith('\\')
            _ls[_i] = ' ' * len(_l)
        else:
            _cont = False
    code = '\n'.join(_ls)
    rlines = src.split('\n'); clines = code.split('\n'); mlines = comm.split('\n')
    base = os.path.basename(path)
    if not re.fullmatch(r'[a-z0-9_]+\.[ch]', base):
        add('4.1.a', rel, 1, 'file name not lowercase/digits/underscore')
    if b'\r\n' in raw:
        add('3.6.a', rel, 1, 'CR-LF line endings')
    if not re.search(r'/\*+\s*end of file\s*\*+/\s*$', src):
        add('3.3.c', rel, len(rlines), 'no end-of-file marker comment')
    for k, l in enumerate(rlines, 1):
        if len(l.rstrip('\r')) > 80:
            add('1.2.a', rel, k, f'{len(l)} chars')
        if '\t' in l:
            add('3.5.a', rel, k, 'tab character')
    # comments
    for m in re.finditer(r'/\*(.*?)\*/|//([^\n]*)', src, re.S):
        body = m.group(1) if m.group(1) is not None else m.group(2)
        ln = lineno(src, m.start())
        inner = body
        if '/*' in inner or ('//' in inner and 'http' not in inner):
            add('2.1.b', rel, ln, 'comment contains /* or //')
        for sub in inner.split('\n'):
            s = sub.strip().lstrip('*').strip()
            if re.fullmatch(r'(?:[a-z_][\w\->\.\[\]]*\s*(?:=[^=].*|\(.*\))\s*;|(?:if|for|while)\s*\(.*\)\s*\{?|return\b.*;|\}\s*(?:else)?\s*\{?)', s):
                add('2.1.c', rel, ln, 'looks like commented-out code: ' + s)
                break
    # preprocessor
    if path.endswith('.h'):
        pp = [l for l in pcode.split('\n') if l.strip()]
        if not (len(pp) >= 2 and re.match(r'\s*#\s*(ifndef|if\s+!\s*defined)', pp[0]) and re.match(r'\s*#\s*define', pp[1])):
            add('4.2.b', rel, 1, 'no include guard at top')
    else:
        stem = base[:-2]
        hdr_exists = os.path.exists(path[:-2] + '.h')
        if hdr_exists and not re.search(r'#\s*include\s*"(?:[\w./]*/)?' + re.escape(stem) + r'\.h"', src):
            add('4.3.c', rel, 1, f'does not include {stem}.h')
    for k, l in enumerate(pcode.split('\n'), 1):
        m = re.match(r'\s*#\s*define\s+(\w+)\(', l)
        if m:
            add('6.3.a', rel, k, 'function-like macro ' + m.group(1))
        m = re.match(r'\s*#\s*define\s+(\w*[a-z]\w*)\b', l)
        if m:
            add('6.1.f', rel, k, 'macro name has lowercase: ' + m.group(1))
        if re.match(r'\s*#\s*include\s*["<]\s*/', l) or re.match(r'\s*#\s*include\s*"[^"]*\.c"', l):
            add('4.3.d/f', rel, k, 'absolute path or .c include')
    # tokens
    for m in re.finditer(r'\b(goto|continue|register|auto)\b', code):
        r = {'goto': '1.7.c', 'continue': '1.7.d', 'register': '1.7.b', 'auto': '1.7.a'}[m.group(1)]
        add(r, rel, lineno(code, m.start()), m.group(1))
    for m in re.finditer(r'\b(abort|exit|setjmp|longjmp)\s*\(', code):
        add('8.5.b', rel, lineno(code, m.start()), m.group(1))
    for m in re.finditer(r'\b(short|long)\b', code):
        add('5.2.b', rel, lineno(code, m.start()), m.group(1))
    for m in re.finditer(r'\b(float|double)\b(?!\w)', code):
        ln = lineno(code, m.start())
        if not re.search(r'typedef', clines[ln - 1]):
            add('5.4.b.i', rel, ln, 'plain ' + m.group(1))
    for m in re.finditer(r'\b(BOOL|TRUE|FALSE)\b', code):
        add('5.6.a', rel, lineno(code, m.start()), m.group(1))
    for m in re.finditer(r'(?<![\w.])(int|unsigned|signed)\b(?!\s*\w*_t)', code):
        ln = lineno(code, m.start()); l = clines[ln - 1]
        if re.search(r'\b(int|unsigned|signed)\s+main\b|usermain', l):
            continue
        add('5.2.a', rel, ln, 'plain ' + m.group(1) + ': ' + l.strip())
    for m in re.finditer(r'(?<![\w])char\b', code):
        ln = lineno(code, m.start()); l = clines[ln - 1]
        if not re.search(r'char\s*(?:const\s*)?\*|char\s+\w+\s*\[|\bchar\b\s*\)', l):
            add('5.2.c', rel, ln, 'char used for non-string data?: ' + l.strip())
    # 3.1 white space
    for k, l in enumerate(clines, 1):
        if l.lstrip().startswith('#'):
            continue
        for mm in re.finditer(r'\b(if|while|for|switch|return)(\(|\s{2,}\S)', l):
            add('3.1.a', rel, k, l.strip())
        for mm in re.finditer(r',(?=[^\s])', l):
            add('3.1.k', rel, k, l.strip()); break
        if re.search(r'\S\s+;', l) and not re.search(r'for\s*\(', l):
            add('3.1.m', rel, k, l.strip())
        if re.search(r'[^\s=!<>+\-*/%&|^]=[^=\s]|[^\s]=[^=]|[^=!<>+\-*/%&|^\s]=\S', l) and not re.search(r'[=!<>]=', l) and '[' not in l:
            pass
        if re.search(r'\w(\+=|-=|\*=|/=|%=|&=|\|=|\^=)|(\+=|-=|\*=|/=|%=|&=|\|=|\^=)\w', l):
            add('3.1.b', rel, k, l.strip())
        if re.search(r'\w\s*(?<![=!<>+\-*/%&|^])=(?!=)\S|\S(?<![=!<>+\-*/%&|^\s])=(?!=)', l):
            if not re.search(r'\boperator\b', l):
                add('3.1.b', rel, k, l.strip())
        if re.search(r'^\s*\S', l):
            ind = len(l) - len(l.lstrip(' '))
            if ind % 4 and not re.match(r'\s*[*&|+\-?:"(]|\s*[\w\)\]]+[,)]', l):
                pass  # alignment continuation lines are allowed; too noisy
        for mm in re.finditer(r'\b(\d+[uU]?|[A-Z][A-Z0-9_]+)\s*==\s*[a-z_]', l):
            pass
        for mm in re.finditer(r'([\w\]\)]+)\s*(==)\s*(-?(?:0x[0-9a-fA-F]+|\d+(?:\.\d*)?)[uUlLfF]*|[A-Z][A-Z0-9_]{1,}|true|false|NULL)\b(?!\s*[\(\[\.])', l):
            lhs = mm.group(1)
            if re.fullmatch(r'-?(?:0x[0-9a-fA-F]+|\d+)[uUlL]*|[A-Z][A-Z0-9_]+|true|false|NULL', lhs):
                continue
            add('8.6.a', rel, k, l.strip())
        if re.search(r'\bwhile\s*\(\s*(1|true|TRUE)\s*\)', l):
            add('8.4.c', rel, k, l.strip())
    # 1.3.a/b, 8.2.c, 8.2.d, 8.4.b
    for m in re.finditer(r'\b(if|for|while|switch)\s*\(', code):
        kw = m.group(1); ln = lineno(code, m.start())
        p = code.index('(', m.start()); e = match_paren(code, p)
        cond = code[p + 1:e - 1]
        rest = code[e:]
        nxt = re.match(r'\s*', rest).end()
        ch = rest[nxt:nxt + 1]
        if kw == 'while':
            before = code[:m.start()].rstrip()
            if before.endswith('}') and ch == ';':
                continue  # do..while
        if kw in ('if', 'while') and re.search(r'(?<![=!<>+\-*/%&|^])=(?![=])', cond):
            add('8.2.c' if kw == 'if' else '8.4.b', rel, ln, 'assignment in condition: ' + ' '.join(cond.split())[:80])
        if ch != '{':
            if not (kw in ('for', 'while') and ch == ';'):
                add('1.3.a', rel, ln, f'{kw} body without braces')
            elif kw in ('for', 'while') and ch == ';':
                add('8.4.d', rel, ln, f'empty {kw} body with ;')
        else:
            # brace on same line?
            if '\n' not in rest[:nxt]:
                add('1.3.b', rel, ln, f'{{ on same line as {kw}')
        if kw in ('if', 'while') and ('&&' in cond or '||' in cond):
            # top-level operand split
            d = 0; parts = []; cur = ''
            i = 0
            while i < len(cond):
                c = cond[i]
                if c == '(': d += 1
                if c == ')': d -= 1
                if d == 0 and cond[i:i + 2] in ('&&', '||'):
                    parts.append(cur); cur = ''; i += 2; continue
                cur += c; i += 1
            parts.append(cur)
            for part in parts:
                s = part.strip()
                if s.startswith('!'):
                    s = s[1:].strip()
                if re.fullmatch(r'[\w.\->\[\]]+', s):
                    continue
                if s.startswith('(') and match_paren(s, 0) == len(s):
                    continue
                if re.fullmatch(r'[\w.\->]+\s*\(.*\)', s, re.S):
                    continue  # a function call: single operand (inferred acceptable)
                add('1.4.b', rel, ln, 'unparenthesised && / || operand: ' + ' '.join(s.split())[:60])
                break
    for m in re.finditer(r'\belse\b', code):
        rest = code[m.end():]
        nxt = re.match(r'\s*', rest).end()
        if rest[nxt:nxt + 2] == 'if':
            continue
        if rest[nxt:nxt + 1] != '{':
            add('1.3.a', rel, lineno(code, m.start()), 'else body without braces')
        elif '\n' not in rest[:nxt]:
            add('1.3.b', rel, lineno(code, m.start()), '{ on same line as else')
    # else-if chain must end in else
    for m in re.finditer(r'\belse\s+if\s*\(', code):
        # find end of this else-if body and see whether chain continues
        p = code.index('(', m.start() + 4); e = match_paren(code, p)
        rest = code[e:]; nxt = re.match(r'\s*', rest).end()
        if rest[nxt:nxt + 1] != '{':
            continue
        i = e + nxt; d = 0
        for j in range(i, len(code)):
            if code[j] == '{': d += 1
            elif code[j] == '}':
                d -= 1
                if d == 0:
                    break
        after = code[j + 1:]
        if not re.match(r'\s*else\b', after):
            add('8.2.d', rel, lineno(code, m.start()), 'else-if chain without final else')
    # switch default
    for m in re.finditer(r'\bswitch\s*\(', code):
        p = code.index('(', m.start()); e = match_paren(code, p)
        i = code.find('{', e); d = 0
        for j in range(i, len(code)):
            if code[j] == '{': d += 1
            elif code[j] == '}':
                d -= 1
                if d == 0: break
        body = code[i:j]
        if not re.search(r'\bdefault\s*:', body):
            add('8.3.b', rel, lineno(code, m.start()), 'switch without default')
    # struct/typedef names
    for m in re.finditer(r'\btypedef\b[^;{]*?(\{)?', code):
        # find the typedef name: last identifier before ';' at depth 0
        i = m.start(); d = 0
        for j in range(i, len(code)):
            if code[j] in '{(': d += 1
            elif code[j] in '})': d -= 1
            elif code[j] == ';' and d == 0: break
        seg = code[i:j]
        mm = re.search(r'(\w+)\s*(?:\[[^\]]*\])?\s*$', seg)
        fp = re.search(r'\(\s*\*\s*(\w+)\s*\)', seg)
        name = fp.group(1) if fp and '{' not in seg else (mm.group(1) if mm else '?')
        if not re.fullmatch(r'[a-z][a-z0-9_]*_t', name):
            add('5.1.a', rel, lineno(code, i), 'typedef name ' + name)
    for m in re.finditer(r'\b(struct|union|enum)\s+(\w+)?\s*\{', code):
        before = code[max(0, m.start() - 40):m.start()]
        if not re.search(r'typedef\s*$', before):
            add('5.1.b', rel, lineno(code, m.start()), f'{m.group(1)} {m.group(2) or ""} not typedef-ed')
    # functions: definitions at depth 0
    depth = 0; i = 0; n = len(code)
    func_re = re.compile(r'(?:^|\n)([ \t]*(?:static\s+|inline\s+|extern\s+|const\s+|volatile\s+|__attribute__\s*\(\(.*?\)\)\s*)*[\w\s\*]+?\b(\w+)\s*)\(', re.S)
    # simple scanner: walk top level, find '{' at depth 0 preceded by ')'
    pos = 0; topstart = 0
    while pos < n:
        c = code[pos]
        if c == '{':
            if depth == 0:
                head = code[topstart:pos]
                hm = re.search(r'(\w+)\s*\(([^;{}]*)\)\s*$', head, re.S)
                is_func = bool(hm) and not re.search(r'=\s*$|\b(struct|union|enum)\b[^;(]*$', head.split(';')[-1]) and hm.group(1) not in KW
                # body end
                d = 0
                for j in range(pos, n):
                    if code[j] == '{': d += 1
                    elif code[j] == '}':
                        d -= 1
                        if d == 0: break
                if is_func:
                    fname = hm.group(1); hdr = head.split(';')[-1].split('}')[-1]
                    fl = lineno(code, pos); el = lineno(code, j)
                    length = el - fl + 1
                    if length > 100:
                        add('6.2.a', rel, fl, f'{fname}() body {length} lines')
                    body = code[pos:j + 1]
                    nret = len(re.findall(r'\breturn\b', body))
                    if nret > 1:
                        add('6.2.c', rel, fl, f'{fname}() has {nret} returns')
                    if re.search(r'[A-Z]', fname) and fname != 'usermain':
                        add('6.1.e', rel, fl, f'{fname}() has uppercase')
                    if len(fname) > 31:
                        add('6.1.d', rel, fl, f'{fname}() {len(fname)} chars')
                    if fname.startswith('_'):
                        add('6.1.c', rel, fl, fname)
                    static = re.search(r'\bstatic\b', hdr)
                    if not static and path.endswith('.c') and fname not in ('main', 'usermain') \
                            and not re.search(r'\b' + re.escape(fname) + r'\s*\(', all_headers_text):
                        add('6.2.d/e', rel, fl, f'{fname}() non-static but no prototype in any header')
                    if re.search(r'(cyclic|handler|irq|callback)', fname) and not fname.endswith('_isr') and not static:
                        add('6.5.b', rel, fl, f'{fname}() looks like a handler, not *_isr (inferred)')
                    # params
                    params = hm.group(2)
                    for prm in params.split(','):
                        prm = prm.strip()
                        if prm in ('void', ''): continue
                        pm = re.search(r'(\**)\s*(\w+)\s*(\[[^\]]*\])?\s*$', prm)
                        if not pm: continue
                        nm = pm.group(2); isptr = '*' in prm or pm.group(3)
                        PARAMS.append((rel, fl, nm, bool(isptr), prm))
                    # locals
                    agg=[]
                    for am in re.finditer(r'\b(struct|union|enum)\b[^;{}()]*\{', body):
                        b0=am.end()-1; dd2=0
                        for b1 in range(b0,len(body)):
                            if body[b1]=='{': dd2+=1
                            elif body[b1]=='}':
                                dd2-=1
                                if dd2==0: break
                        agg.append((b0,b1))
                    for dm in re.finditer(r'(?:^|[;{}])\s*((?:static\s+|const\s+|volatile\s+)*)(' + SCALAR + r'|struct\s+\w+|[a-z_]\w*_t)\b((?:\s*\*\s*(?:const\s+)?)*)\s*(\w+)\s*(\[[^\]]*\])?\s*(=|;|,)', body):
                        nm = dm.group(4)
                        if nm in KW: continue
                        if any(a0 < dm.start(4) < a1 for a0,a1 in agg): continue
                        ln2 = fl + body.count('\n', 0, dm.start(4))
                        F_locals = (rel, ln2, nm, bool(dm.group(3).strip()) or False, dm.group(2), dm.group(6), dm.group(5))
                        LOCALS.append(F_locals)
                    pass
                pos = j + 1 if is_func else pos + 1
                if is_func:
                    topstart = pos
                    continue
            depth += 1
        elif c == '}':
            depth -= 1
            if depth == 0:
                topstart = pos + 1
        elif c == ';' and depth == 0:
            stmt = code[topstart:pos]
            GLOBALS_.append((rel, lineno(code, topstart + len(stmt) - len(stmt.lstrip())), stmt))
            topstart = pos + 1
        pos += 1


def name_checks(rel, ln, nm, isptr, typ, kind, isglobal=False):
    if re.search(r'[A-Z]', nm):
        add('7.1.f', rel, ln, f'{kind} {nm} has uppercase')
    if len(nm) < 3:
        add('7.1.e', rel, ln, f'{kind} {nm} shorter than 3')
    if len(nm) > 31:
        add('7.1.d', rel, ln, f'{kind} {nm} longer than 31')
    stem = nm
    if kind in ('file-static', 'file-scope const') and re.match(r'[gs]_', stem):
        stem = stem[2:]
    if isglobal:
        if not stem.startswith('g_'):
            add('7.1.j', rel, ln, f'global {nm} lacks g_')
        else:
            stem = stem[2:]
    if isptr == 'pp':
        if not stem.startswith('pp_'):
            add('7.1.l', rel, ln, f'{kind} {nm} pointer-to-pointer lacks pp_')
        stem = stem[3:] if stem.startswith('pp_') else stem
    elif isptr == 'arr':
        pass
    elif isptr:
        if not stem.startswith('p_'):
            add('7.1.k', rel, ln, f'{kind} {nm} pointer lacks p_')
        stem = stem[2:] if stem.startswith('p_') else stem
    if typ in ('bool', 'BOOL') and not stem.startswith('b_') and isptr in (False, None, '', 'arr'):
        add('7.1.m', rel, ln, f'{kind} {nm} bool lacks b_')

for rel, ln, nm, isptr, typ, term, arr in LOCALS:
    if arr:
        isptr = 'arr'
    name_checks(rel, ln, nm, isptr, typ.strip(), 'local')
for rel, ln, nm, isptr, prm in PARAMS:
    typ = 'bool' if re.match(r'\s*(const\s+)?bool\b', prm) else ''
    ip = 'pp' if prm.count('*') >= 2 else isptr
    name_checks(rel, ln, nm, ip, typ, 'param')
for rel, ln, stmt in GLOBALS_:
    s = ' '.join(stmt.split())
    if s.startswith('#') or not s or re.match(r'(typedef|struct\s+\w+\s*$|enum|union)', s):
        continue
    s = re.sub(r'^.*#\s*\w+[^\n]*', '', s).strip()
    s2 = re.sub(r'\{.*\}', '{}', s)
    if '(' in s2.split('=')[0] and not re.search(r'\(\s*\*\s*\w+\s*\)', s2.split('=')[0]):
        continue  # prototype
    m = re.match(r'((?:static|extern|const|volatile|\s)*)\s*(' + SCALAR + r'|struct\s+\w+|[a-z_]\w*_t|\w+)\b\s*((?:\*\s*(?:const\s+)?)*)\s*(\w+)\s*(\[[^\]]*\])*\s*(=.*)?$', s2)
    if not m:
        continue
    quals, typ, stars, nm = m.group(1), m.group(2), m.group(3), m.group(4)
    if nm in KW or typ in ('return',):
        continue
    is_static = 'static' in quals
    is_extern = 'extern' in quals
    is_const = 'const' in quals
    if rel.endswith('.h') and not is_extern:
        add('4.2.c', rel, ln, 'variable storage defined in header: ' + s[:70])
    if rel.endswith('.c') and is_extern:
        add('4.2.c*', rel, ln, 'extern declaration inside .c file: ' + nm)
    if is_const and not stars:
        name_checks(rel, ln, nm, stars.count('*') >= 2 and 'pp' or bool(stars.strip()), typ, 'file-scope const', False)
        continue
    # BARR-C 7.1.j: "global" = visible outside module.  File-static treated as module-local (team convention); report separately
    if is_static:
        name_checks(rel, ln, nm, 'pp' if stars.count('*') >= 2 else bool(stars.strip()), typ, 'file-static', False)
    else:
        name_checks(rel, ln, nm, 'pp' if stars.count('*') >= 2 else bool(stars.strip()), typ, 'global', True)


# 8.1.a / 3.3.a second pass
for path in files:
    rel = os.path.relpath(path, ROOT)
    code = strip(open(path, encoding='utf-8', errors='replace').read())[0]
    ls = code.split('\n')
    for k, l in enumerate(ls, 1):
        if l.lstrip().startswith('#'):
            continue
        if re.match(r'\s*(?:static\s+|const\s+|volatile\s+)*(?:' + SCALAR + r'|struct\s+\w+)\s+\**\s*\w+(?:\s*\[[^\]]*\])?(?:\s*=\s*[^,;(){}]+)?\s*,\s*\**\s*[a-zA-Z_]\w*\s*(?:=|,|;|\[)', l):
            add('8.1.a', rel, k, 'multiple declarators: ' + l.strip()[:80])
        t = re.sub(r'\bfor\s*\(.*\)', '', l)
        if t.count(';') >= 2:
            add('3.3.a', rel, k, 'more than one statement: ' + l.strip()[:80])


def load_allow(path):
    rules = []
    for raw in open(path, encoding='utf-8'):
        if (not raw.strip()) or raw.startswith('#'):
            continue
        cols = raw.rstrip('\n').split('\t')
        rules.append((cols[0], cols[1], re.compile(cols[2]), cols[3] if len(cols) > 3 else ''))
    return rules


if ALLOW is None:
    for r in F:
        print('\t'.join(map(str, r)))
else:
    allow = load_allow(ALLOW)
    used = set()
    left = []
    for r in F:
        hit = None
        for k, (rule, glob, rx, _why) in enumerate(allow):
            if (rule == r[0]) and fnmatch.fnmatch(r[1], glob) and rx.search(r[3]):
                hit = k
                break
        if hit is None:
            left.append(r)
        else:
            used.add(hit)
    for r in left:
        print('\t'.join(map(str, r)))
    for k, (rule, glob, rx, why) in enumerate(allow):
        if k not in used:
            print('note: allowlist line not used any more: %s %s %s' % (rule, glob, rx.pattern), file=sys.stderr)
    print('barrc: %d findings, %d allowed, %d not allowed' % (len(F), len(F) - len(left), len(left)), file=sys.stderr)
    sys.exit(1 if left else 0)
