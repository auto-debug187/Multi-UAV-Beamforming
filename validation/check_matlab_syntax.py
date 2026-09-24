"""Static grammar check only. pip install tree-sitter tree-sitter-matlab"""
from pathlib import Path
import json
from tree_sitter import Language, Parser
import tree_sitter_matlab

root=Path(__file__).resolve().parents[1]
parser=Parser(Language(tree_sitter_matlab.language()))
errors=[]; files=sorted(root.rglob('*.m'))
for p in files:
    tree=parser.parse(p.read_bytes()); stack=[tree.root_node]
    while stack:
        node=stack.pop()
        if node.type=='ERROR' or node.is_missing:
            errors.append(dict(file=str(p.relative_to(root)),line=node.start_point.row+1,type=node.type))
        stack.extend(node.children)
result=dict(check='Static MATLAB grammar only; not MATLAB execution',files_checked=len(files),errors=errors)
(root/'validation'/'matlab_syntax_check.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result,indent=2)); assert not errors
