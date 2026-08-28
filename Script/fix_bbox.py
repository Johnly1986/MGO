#!/usr/bin/env python3
import sys

with open('TilesConverter/TilesConverter.cpp', 'r', encoding='utf-8') as f:
    content = f.read()

start_marker = '// Compute union of all valid children'
start_idx = content.find(start_marker)
if start_idx == -1:
    print("ERROR: Could not find start marker")
    sys.exit(1)

end_marker = 'static void writeBoxJson'
end_idx = content.find(end_marker, start_idx)
if end_idx == -1:
    print("ERROR: Could not find end marker")
    sys.exit(1)

# Find the last } before writeBoxJson (the closing of UpdateGridCellBBoxes)
block = content[start_idx:end_idx]
last_brace = block.rfind('}')
replace_end = start_idx + last_brace + 1

new_block = """// All bboxes are now in ECEF space (vertices converted to absolute ECEF
    // in GroupCellByMaterial). Simple union of children's bboxes.
    bool first = true;
    for (auto& c : cell.children)
    {
        if (!c) continue;
        if (!c->hasContent && !c->hasChildren() && c->materialGroups.empty()) continue;

        if (first)
        {
            for (int a = 0; a < 3; ++a)
            {
                cell.bboxMin[a]      = c->bboxMin[a];
                cell.bboxMax[a]      = c->bboxMax[a];
                cell.localBboxMin[a] = c->localBboxMin[a];
                cell.localBboxMax[a] = c->localBboxMax[a];
            }
            first = false;
        }
        else
        {
            for (int a = 0; a < 3; ++a)
            {
                if (c->bboxMin[a] < cell.bboxMin[a])      cell.bboxMin[a]      = c->bboxMin[a];
                if (c->bboxMax[a] > cell.bboxMax[a])      cell.bboxMax[a]      = c->bboxMax[a];
                if (c->localBboxMin[a] < cell.localBboxMin[a]) cell.localBboxMin[a] = c->localBboxMin[a];
                if (c->localBboxMax[a] > cell.localBboxMax[a]) cell.localBboxMax[a] = c->localBboxMax[a];
            }
        }
    }
}"""

content = content[:start_idx] + new_block + content[replace_end:]

with open('TilesConverter/TilesConverter.cpp', 'w', encoding='utf-8') as f:
    f.write(content)

print("SUCCESS: UpdateGridCellBBoxes simplified")
