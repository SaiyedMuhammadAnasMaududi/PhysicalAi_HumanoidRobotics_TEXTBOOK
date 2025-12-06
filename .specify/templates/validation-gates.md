# Validation Gates

**Purpose**: Define quality gates that each chapter must pass before being marked complete.

**Last Updated**: 2025-12-05

---

## Overview

Every chapter must pass **ALL FIVE validation gates** before publication:

1. ✅ **Source Check** - All claims verified with cited sources
2. ✅ **Clarity Check** - Readability verified (Flesch-Kincaid 8-10)
3. ✅ **Code Check** - All examples tested and functional
4. ✅ **Build Check** - Chapter integrates without build errors
5. ✅ **Consistency Check** - Formatting and terminology aligned

**Failure in ANY gate = Chapter NOT ready for publication**

---

## Gate 1: Source Check ✅

**Purpose**: Ensure academic rigor and verifiable claims

###Criteria

- [ ] **Minimum 40% Academic Sources**
  - Research papers (IEEE, ACM, arXiv with high citations)
  - Peer-reviewed textbooks
  - Conference proceedings (ICRA, IROS, CVPR, NeurIPS, etc.)

- [ ] **All URLs Verified**
  - No broken links
  - All DOIs accessible
  - Documentation links current

- [ ] **Proper Citation Format**
  - APA or IEEE format consistently applied
  - In-text citations match reference list
  - All sources in reference list are cited in text

- [ ] **No Unsupported Claims**
  - Technical definitions sourced
  - Statistics and data cited
  - Algorithm descriptions referenced

### Validation Process

```bash
# Run source verification
python3 scripts/verify_sources.py docs/module-name/chapter.md

# Manual checks
1. Open each URL in reference list
2. Verify accessibility
3. Check citation format
4. Count academic vs. non-academic sources
5. Calculate percentage
```

### Pass Criteria

- Academic sources ≥ 40% of total
- All URLs accessible (status 200)
- Zero unsourced technical claims
- Citation format consistent (no mix of APA/IEEE)

### Example

**PASS** ✅:
```markdown
## References

1. Thrun, S., Burgard, W., & Fox, D. (2005). Probabilistic robotics. MIT Press.
2. Siciliano, B., & Khatib, O. (2016). Springer handbook of robotics...
3. Open Robotics. (2023). ROS 2 Humble documentation...

Academic: 2/3 = 67% ✅
All URLs verified ✅
```

**FAIL** ❌:
```markdown
## References

1. Some blog post
2. Wikipedia article
3. YouTube video

Academic: 0/3 = 0% ❌ (Need ≥ 40%)
```

---

## Gate 2: Clarity Check ✅

**Purpose**: Ensure content is accessible to target audience (grade 8-10 reading level)

### Criteria

- [ ] **Flesch-Kincaid Grade Level: 8-10**
  - Not too simple (< 8 = may seem condescending)
  - Not too complex (> 10 = may confuse beginners)

- [ ] **Jargon Defined**
  - All technical terms introduced before use
  - Acronyms spelled out on first use
  - Glossary terms linked when appropriate

- [ ] **Clear Structure**
  - Logical flow from simple to complex
  - Transitions between sections
  - Each section has clear purpose

- [ ] **Student-Focused Language**
  - Active voice preferred
  - Direct address ("you will learn")
  - Concrete examples before abstractions

### Validation Process

```bash
# Automated readability check
npm run readability-check docs/module-name/chapter.md

# Or use online tools
https://readabilityformulas.com/free-readability-formula-tests.php
```

### Pass Criteria

- Flesch-Kincaid Grade: 8.0-10.0
- Flesch Reading Ease: 50-70 (Fairly Difficult to Standard)
- All jargon defined on first use
- No undefined acronyms

### Example Scoring

**Content Sample**:
> "ROS 2 (Robot Operating System 2) provides a framework for robot software development. It handles communication between different parts of your robot using a publish-subscribe pattern. Think of it like different people in a team passing messages to coordinate their work."

**Flesch-Kincaid**: 9.2 ✅ (within 8-10 range)
**Reading Ease**: 62 ✅ (Standard difficulty)

---

## Gate 3: Code Check ✅

**Purpose**: Ensure all code examples are tested, functional, and reproducible

### Criteria

- [ ] **All Code Examples Tested**
  - Run in specified environment
  - Produce expected output
  - No runtime errors

- [ ] **Dependencies Documented**
  - Package versions specified
  - Installation instructions clear
  - Environment requirements listed

- [ ] **Inline Comments Present**
  - Explain educational concepts
  - Not just describing what code does
  - Highlight key learning points

- [ ] **Expected Output Documented**
  - Show what students should see
  - Explain any warnings (if expected)
  - Provide troubleshooting for common issues

### Validation Process

```bash
# Manual validation for each code block
1. Extract code from chapter markdown
2. Setup clean test environment
3. Install dependencies
4. Run code
5. Verify output matches documentation
6. Document validation status
```

### Pass Criteria

- 100% of code examples run without errors
- All dependencies documented with versions
- Expected output matches actual output
- Setup instructions reproducible by fresh user

### Example

**PASS** ✅:
````markdown
```python
# Code with inline comments explaining concepts
import rclpy  # ROS 2 Python library
...
```

**Prerequisites**: ROS 2 Humble, Python 3.8+
**Expected Output**: `[INFO] Publishing: "Hello ROS 2: 0"`
**Tested**: 2025-12-05, Python 3.10, Ubuntu 22.04 ✅
````

**FAIL** ❌:
````markdown
```python
# Code without comments or setup instructions
import mystery_package  # Not in requirements
```
❌ No dependencies documented
❌ Not tested
````

---

## Gate 4: Build Check ✅

**Purpose**: Ensure chapter integrates smoothly into Docusaurus site

### Criteria

- [ ] **Valid Frontmatter**
  - All required YAML fields present
  - Correct syntax (no parsing errors)
  - Sidebar position set appropriately

- [ ] **No Broken Internal Links**
  - All `[Link Text](./other-page)` resolve
  - All `![Image](path/to/image)` exist
  - All cross-references work

- [ ] **Images Load Correctly**
  - All images in `static/` directory
  - Paths correct (relative to docs/)
  - File sizes reasonable (< 500KB per image)

- [ ] **Build Succeeds**
  - `npm run build` completes
  - No warnings or errors
  - Build time < 5 minutes

### Validation Process

```bash
# Run Docusaurus build
npm run build

# Check for warnings
grep -i "warning" build.log

# Check for broken links
npx broken-link-checker http://localhost:3000 --recursive

# Verify file sizes
find static/img -type f -size +500k
```

### Pass Criteria

- `npm run build` exits with code 0
- Zero broken links
- Zero image load failures
- All images < 500KB
- Total build size < 50 MB

### Example Output

**PASS** ✅:
```
✔ Client bundle compiled successfully
✔ Server bundle compiled successfully
✔ Static files copied
Success! Generated static files in build/
Build time: 2.3 minutes ✅
Total size: 35 MB ✅
```

**FAIL** ❌:
```
✖ Error: Cannot resolve '../invalid-link.md'
❌ Broken link found
```

---

## Gate 5: Consistency Check ✅

**Purpose**: Ensure formatting and terminology align with constitution and other chapters

### Criteria

- [ ] **Frontmatter Consistent**
  - Same fields as other chapters
  - Keywords follow project conventions
  - Sidebar position logical

- [ ] **Terminology Consistent**
  - Use glossary terms consistently
  - No switching between synonyms
  - Acronyms used correctly

- [ ] **Formatting Consistent**
  - Heading levels follow hierarchy (H2, H3, not skipping)
  - Code blocks use language identifiers
  - Callout boxes (notes, tips, warnings) used appropriately

- [ ] **Learning Objectives Format**
  - Use action verbs (understand, implement, apply)
  - Measurable outcomes
  - Match summary takeaways

### Validation Process

```bash
# Automated checks
scripts/check_frontmatter.py docs/module-name/*.md
scripts/check_terminology.py docs/module-name/*.md

# Manual review
1. Compare frontmatter to template
2. Check heading hierarchy
3. Verify code block language tags
4. Review terminology against glossary
```

### Pass Criteria

- Frontmatter matches template structure
- All headings follow H1→H2→H3 hierarchy (no skips)
- Terminology matches glossary
- Formatting matches other chapters in module

### Example

**PASS** ✅:
```yaml
---
title: "ROS 2 Basics"  ✅ Clear title
keywords: ["ros2", "nodes", "topics"]  ✅ Lowercase, consistent
learning_objectives:  ✅ Field present
  - "Understand ROS 2 architecture"  ✅ Action verb
  - "Create basic ROS 2 nodes"  ✅ Measurable
---
```

**FAIL** ❌:
```yaml
---
title: "basics"  ❌ Not descriptive
keywords: ROS, Nodes  ❌ Wrong format (not array)
# Missing learning_objectives ❌
---
```

---

## Validation Workflow Summary

### For Each Chapter

1. **Author completes draft**
2. **Run Gate 1: Source Check**
   - Verify sources, count academic percentage
3. **Run Gate 2: Clarity Check**
   - Check readability score
4. **Run Gate 3: Code Check**
   - Test all code examples
5. **Run Gate 4: Build Check**
   - Run `npm run build`, check for errors
6. **Run Gate 5: Consistency Check**
   - Compare to template and other chapters

7. **If ALL gates PASS**: Mark chapter complete ✅
8. **If ANY gate FAILS**: Document issues, revise, re-validate

### Validation Checklist Template

```markdown
## Chapter Validation: [Module X - Chapter Y]

**Date**: 2025-12-05
**Validator**: [Name]

- [ ] Gate 1: Source Check
  - Academic sources: X/Y = Z% (need ≥ 40%)
  - All URLs verified: Yes/No
  - Citation format: Consistent/Issues
  - Status: PASS/FAIL

- [ ] Gate 2: Clarity Check
  - Flesch-Kincaid: X.X (need 8.0-10.0)
  - Reading Ease: XX (need 50-70)
  - Jargon defined: Yes/No
  - Status: PASS/FAIL

- [ ] Gate 3: Code Check
  - Examples tested: X/Y
  - Dependencies documented: Yes/No
  - Output verified: Yes/No
  - Status: PASS/FAIL

- [ ] Gate 4: Build Check
  - Build succeeded: Yes/No
  - Broken links: X
  - Images load: Yes/No
  - Status: PASS/FAIL

- [ ] Gate 5: Consistency Check
  - Frontmatter complete: Yes/No
  - Terminology consistent: Yes/No
  - Formatting matches: Yes/No
  - Status: PASS/FAIL

**Overall Status**: PASS / FAIL
**Notes**: [Any issues or comments]
```

---

## Automated Validation (Future Enhancement)

```bash
# Run all validation gates
./scripts/validate-chapter.sh docs/module-01-ros2/02-ros2-basics.md

# Output
✅ Gate 1: Source Check - PASS (45% academic, all URLs valid)
✅ Gate 2: Clarity Check - PASS (FK: 9.1, RE: 61)
✅ Gate 3: Code Check - PASS (3/3 examples tested)
✅ Gate 4: Build Check - PASS (build succeeded, 0 warnings)
✅ Gate 5: Consistency Check - PASS (all criteria met)

🎉 Chapter READY for publication
```

---

**Remember**: These gates exist to ensure quality and consistency across the entire textbook. Every chapter must pass ALL gates before being marked complete.
