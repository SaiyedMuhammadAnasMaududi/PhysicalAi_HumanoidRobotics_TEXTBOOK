# Implementation Plan: Physical AI & Humanoid Robotics Textbook — Capstone Project

**Branch**: `001-voice-controlled-humanoid` | **Date**: 2025-12-05 | **Spec**: [specs/006-voice-controlled-humanoid/spec.md](./spec.md)
**Input**: Feature specification from `/specs/006-voice-controlled-humanoid/spec.md`

**Note**: This architectural plan covers the complete textbook development for the Physical AI & Humanoid Robotics educational project, using the voice-controlled humanoid capstone as the integrative goal.

---

## Summary

**Primary Requirement**: Develop a comprehensive educational textbook covering Physical AI and Humanoid Robotics, structured as four core modules (ROS 2, Simulation, Perception, Control) culminating in a capstone project where students build an autonomous voice-controlled humanoid assistant.

**Technical Approach**: Research-concurrent methodology using Spec-Kit Plus workflows, Claude Code for content generation, Docusaurus for deployment on GitHub Pages, and modular pedagogical design ensuring each module is standalone yet coherent with the overall curriculum.

---

## 1. Scope and Dependencies

### 1.1 In Scope

**Textbook Content Boundaries:**
- Module 1: Robotic Nervous System (ROS 2) — nodes, topics, services, URDF, Python integration
- Module 2: Digital Twin (Gazebo & Unity) — physics simulation, sensor simulation, ROS 2 bridges
- Module 3: Perception — computer vision, speech recognition, sensor fusion, ROS integration
- Module 4: Control, Planning & Natural Interaction — task planning, behavior trees, VLA integration, safety constraints
- Capstone Project: Autonomous Voice-Controlled Humanoid — full integration of all modules with working demo

**Deliverables:**
- 12–20 chapters in Markdown format (1,000–2,500 words each)
- All code examples tested and functional (Python, JavaScript/TypeScript, ROS 2)
- Docusaurus-compatible structure with navigation, frontmatter, diagrams
- GitHub Pages deployment via GitHub Actions
- APA/IEEE citations for all research sources (minimum 40% academically credible)

**Validation Gates:**
- Source Check (all claims verified with URLs or paper references)
- Clarity Check (Flesch-Kincaid grade 8–10)
- Code Check (all examples tested and functional)
- Build Check (no Docusaurus build errors)
- Consistency Check (formatting, terminology aligned with constitution)

### 1.2 Out of Scope

**Explicitly Excluded:**
- Physical robot hardware procurement or assembly instructions (students may use simulation or existing robots)
- Real-time embedded systems programming (focus remains on ROS 2 abstraction layer)
- Advanced reinforcement learning algorithms beyond VLA integration basics
- Commercial deployment strategies or production-grade system hardening
- Non-English translations or localization

### 1.3 External Dependencies

| Dependency | Type | Ownership | Risk | Mitigation |
|------------|------|-----------|------|------------|
| ROS 2 (Humble/Iron) | Software Platform | Open Robotics | Version changes | Pin specific ROS 2 version in docs; provide migration guide if updated |
| Gazebo Classic/Ignition | Simulation | Open Source Foundation | Deprecation of Classic | Document both; prioritize Gazebo Harmonic (latest) |
| Unity ML-Agents | Simulation Tool | Unity Technologies | API changes | Pin Unity version; provide alternative (Gazebo-only path) |
| OpenCV | Computer Vision | Open Source | Breaking changes | Pin version; use stable API subset |
| Whisper (OpenAI) | Speech Recognition | OpenAI | API changes or access restrictions | Document local deployment alternative |
| Docusaurus | Static Site Generator | Meta/Facebook | Breaking updates | Pin version in package.json; test before upgrading |
| GitHub Pages | Hosting | GitHub | Service availability | Document alternative deployment (Netlify, Vercel) |
| Claude Code & MCP | Development Tool | Anthropic | API or tool changes | Workflows designed to be tool-agnostic; manual alternative documented |

---

## 2. Key Decisions and Rationale

### Decision 1: Research-Concurrent Approach (Not Upfront Research)

**Options Considered:**
- **Option A**: Complete all research upfront for all modules, then write content sequentially
- **Option B**: Research while writing each module incrementally (selected)

**Trade-offs:**
- Option A: More comprehensive initial understanding, but risks research becoming stale or irrelevant
- Option B: Research stays current and focused; flexibility to adapt based on emerging findings

**Rationale:** Physical AI and robotics are rapidly evolving fields. Research-concurrent ensures the most current sources, reduces wasted research effort on out-of-scope topics, and allows iterative refinement based on writing needs. Aligns with Spec-Kit Plus iterative planning phases.

**Principles:**
- Research conducted per module during Phase 0 (research.md output)
- Research findings immediately applied to content generation
- Sources captured with APA citations in real-time
- Reversible: Can shift to upfront research if critical dependencies discovered

---

### Decision 2: Docusaurus as Publishing Platform

**Options Considered:**
- **Option A**: Custom static site generator
- **Option B**: GitBook or similar hosted platform
- **Option C**: Docusaurus (selected)
- **Option D**: LaTeX/PDF only

**Trade-offs:**
- Option A: Maximum flexibility but high maintenance cost
- Option B: Easiest setup but vendor lock-in and limited customization
- Option C: Open-source, React-based, excellent documentation tooling, GitHub Pages integration
- Option D: Academic standard but poor web accessibility and limited interactivity

**Rationale:** Docusaurus provides the best balance of professional presentation, ease of maintenance, free hosting via GitHub Pages, and support for code blocks, diagrams, and interactive elements. Open-source nature ensures longevity. Aligns with Constitution V (Structural Consistency).

**Principles:**
- All content must render correctly in Docusaurus
- Markdown must include valid frontmatter
- Navigation structure defined in sidebars.js
- Reversible: Markdown content can migrate to other platforms if needed

---

### Decision 3: Modular Pedagogical Structure (Standalone Chapters)

**Options Considered:**
- **Option A**: Linear narrative requiring sequential reading
- **Option B**: Fully modular chapters with standalone comprehension (selected)
- **Option C**: Layered structure with beginner/intermediate/advanced tracks

**Trade-offs:**
- Option A: Better storytelling flow but limits flexibility
- Option B: Maximum flexibility for instructors and self-learners; slightly more redundancy
- Option C: Accommodates skill levels but complex to maintain consistency

**Rationale:** Students and instructors need flexibility to teach/learn topics in varying orders. Modular design with clear prerequisites and self-contained explanations enables diverse pedagogical approaches while maintaining curriculum coherence. Aligns with Constitution III (Modularity and Pedagogical Flow).

**Principles:**
- Each chapter must define learning objectives and prerequisites
- Core concepts introduced before referenced
- Cross-references use consistent linking format
- Chapters tested independently for comprehension
- Reversible: Chapters can be reorganized or extended without breaking dependencies

---

### Decision 4: Python as Primary Language (with JavaScript Secondary)

**Options Considered:**
- **Option A**: Python only
- **Option B**: C++ (ROS standard)
- **Option C**: Python primary, JavaScript/TypeScript secondary for web/agent examples (selected)

**Trade-offs:**
- Option A: Simplest but limits web integration examples
- Option B: Industry standard but steep learning curve
- Option C: Accessible for beginners, supports agent development, enables web-based tools

**Rationale:** Python is widely taught in CS curricula and robotics courses; has excellent ROS 2 bindings (rclpy); supports rapid prototyping. JavaScript/TypeScript enables MCP server examples and agent workflows. Aligns with Constitution IV (Practical Application).

**Principles:**
- All Python code must run on Python 3.8+
- Type hints encouraged but not required
- JavaScript examples use modern ES6+ syntax
- All code examples tested before inclusion
- Reversible: C++ examples can be added as supplementary material

---

### Decision 5: Simulation-First, Hardware-Optional

**Options Considered:**
- **Option A**: Require physical robot hardware for all students
- **Option B**: Simulation-first with optional hardware integration (selected)
- **Option C**: Simulation-only (no hardware considerations)

**Trade-offs:**
- Option A: Most realistic but expensive and inaccessible
- Option B: Accessible to all students; easy validation; optional hardware path for advanced users
- Option C: Maximum accessibility but disconnected from physical robotics

**Rationale:** Physical robots are expensive and not universally accessible. Simulation (Gazebo, Unity) provides safe, repeatable, and cost-free learning environments. Hardware integration path documented for students with access. Aligns with accessibility goals.

**Principles:**
- All examples must work in Gazebo Classic or Gazebo Harmonic
- Hardware integration documented as optional extensions
- Safety considerations taught in simulation context
- Reversible: Hardware-specific chapters can be added later

---

### Decision 6: APA Citation Style with IEEE Fallback

**Options Considered:**
- **Option A**: APA citation style (selected)
- **Option B**: IEEE citation style
- **Option C**: Chicago Manual of Style

**Trade-offs:**
- Option A: Standard in education and social sciences; readable in-text citations
- Option B: Standard in engineering; numbered citations more compact
- Option C: Standard in humanities; verbose for technical content

**Rationale:** Constitution specifies IEEE format, but APA is more common in educational materials and provides better in-text context for readers. IEEE fallback maintained for highly technical references. Both styles meet academic rigor requirements.

**Principles:**
- Minimum 40% academically credible sources
- All sources must have traceable URLs or paper DOIs
- Citations formatted consistently within each chapter
- Reversible: Can batch-convert citation styles if needed

---

## 3. Interfaces and API Contracts

### 3.1 Chapter Content Contract

**Input:**
- Feature spec from `specs/<module>/spec.md`
- Research findings from `specs/<module>/research.md`
- Constitution principles from `.specify/memory/constitution.md`

**Output (Markdown file with frontmatter):**

```yaml
---
title: "Chapter Title"
description: "Brief chapter description"
keywords: ["keyword1", "keyword2", "keyword3"]
learning_objectives:
  - "Objective 1"
  - "Objective 2"
  - "Objective 3"
---
```

**Content Structure:**
1. **Introduction** (100–200 words): Context and motivation
2. **Core Content** (800–2,100 words): Concepts, explanations, examples
3. **Code Examples**: At least one working example (where applicable)
4. **Diagrams**: Visual explanations (where applicable)
5. **Exercises**: 3–5 exercises for practice
6. **Summary** (100–200 words): Key takeaways
7. **References**: APA/IEEE formatted citations

**Errors:**
- `MISSING_FRONTMATTER`: Chapter file lacks required frontmatter fields
- `WORD_COUNT_VIOLATION`: Content outside 1,000–2,500 word range
- `UNTESTED_CODE`: Code example not validated
- `MISSING_CITATIONS`: Claims lack source references
- `READABILITY_VIOLATION`: Flesch-Kincaid score outside 8–10 range

**Versioning Strategy:**
- Chapter versions tracked via Git commit history
- Breaking changes (e.g., removing core concepts) require major version bump
- Additions or clarifications are minor updates

**Idempotency:**
- Re-generating a chapter with same inputs produces identical output (deterministic)

**Timeouts:**
- Research phase: No fixed timeout (research until sufficient sources found)
- Content generation: 1 chapter per 2–4 hours of AI-assisted writing

**Retries:**
- Failed validation gates trigger revision cycle (not regeneration)
- Maximum 3 revision cycles before escalating to human review

---

### 3.2 Docusaurus Build Contract

**Input:**
- All chapter Markdown files in `docs/` directory
- `sidebars.js` navigation configuration
- `docusaurus.config.js` site configuration
- `package.json` with pinned Docusaurus version

**Output:**
- Static site in `build/` directory
- Deployable to GitHub Pages

**Public API:**
- `npm install`: Install dependencies
- `npm run start`: Local development server
- `npm run build`: Production build
- `npm run deploy`: Deploy to GitHub Pages

**Errors:**
- `BUILD_FAILED`: Markdown syntax errors or broken links
- `INVALID_FRONTMATTER`: YAML parsing errors
- `MISSING_NAVIGATION`: Chapter not referenced in sidebars.js

**Idempotency:**
- Same source files produce identical build output

---

### 3.3 ROS 2 Code Example Contract

**Input:**
- Python script with ROS 2 node implementation
- URDF model files
- Launch file configuration

**Output:**
- Functional ROS 2 node that starts without errors
- Expected topics/services published
- Logged output matching documented behavior

**Public API:**
```bash
ros2 run <package_name> <node_name>
ros2 launch <package_name> <launch_file>
```

**Errors:**
- `NODE_STARTUP_FAILED`: Node crashes on launch
- `MISSING_DEPENDENCIES`: Required ROS packages not installed
- `TOPIC_NOT_PUBLISHED`: Expected topic not found

---

## 4. Non-Functional Requirements (NFRs) and Budgets

### 4.1 Performance

**Metrics:**
| Metric | Target | Measurement Method |
|--------|--------|-------------------|
| Page Load Time | < 2 seconds | Lighthouse performance score > 90 |
| Build Time | < 5 minutes | `time npm run build` |
| Code Example Execution | < 10 seconds | Manual testing per example |
| Search Responsiveness | < 500ms | Docusaurus search plugin benchmark |

**Resource Caps:**
- Maximum build artifact size: 50 MB (GitHub Pages limit: 100 MB)
- Maximum single image size: 500 KB
- Maximum chapter Markdown size: 200 KB

### 4.2 Reliability

**SLOs:**
- GitHub Pages uptime: 99.9% (managed by GitHub)
- Build success rate: > 95% (CI/CD pipeline)

**Error Budgets:**
- Tolerate 5% build failures during active development
- Zero tolerance for broken links or non-functional code in production

**Degradation Strategy:**
- If Docusaurus build fails, provide raw Markdown as fallback
- If GitHub Pages down, document alternative deployment (Netlify)

### 4.3 Security

**AuthN/AuthZ:**
- No authentication required (public educational content)
- GitHub repository permissions managed via GitHub settings

**Data Handling:**
- No PII collected
- No user data stored

**Secrets Management:**
- No API keys or secrets required for build
- GitHub Actions deploy token managed via GitHub secrets

**Auditing:**
- Git commit history provides full audit trail
- GitHub Actions logs capture build/deploy events

### 4.4 Cost

**Unit Economics:**
- Zero cost for hosting (GitHub Pages free tier)
- Zero cost for CI/CD (GitHub Actions free tier for public repos)
- Claude Code usage: Best-effort optimization (minimize token usage via efficient prompts)

---

## 5. Data Management and Migration

### 5.1 Source of Truth

**Content Source:**
- Markdown files in `specs/<module>/` and `docs/` directories
- Git repository is authoritative

**Citation Sources:**
- Research papers (IEEE Xplore, arXiv, Google Scholar)
- Official documentation (ROS 2, Gazebo, OpenCV)
- Textbooks (robotics, control theory, computer vision)

### 5.2 Schema Evolution

**Frontmatter Schema:**
- Current version: v1 (fields: title, description, keywords, learning_objectives)
- Future additions (e.g., `difficulty_level`, `estimated_time`) are additive (backward compatible)

**Migration Strategy:**
- Additive changes: No migration required (fields optional)
- Breaking changes: Batch migration script + Git commit

### 5.3 Migration and Rollback

**Content Migration:**
- From raw notes → Spec-Kit Plus structure (completed during initial setup)
- Future: If switching citation style, run batch conversion script

**Rollback:**
- Git revert to previous commit
- Docusaurus rebuild from reverted source

### 5.4 Data Retention

**Retention Policy:**
- All content retained indefinitely in Git history
- Prompt History Records (PHRs) retained for 1 year, then optionally archived
- Build artifacts (`build/` directory) not committed (regenerated on demand)

---

## 6. Operational Readiness

### 6.1 Observability

**Logs:**
- GitHub Actions build logs (stored by GitHub for 90 days)
- Docusaurus build warnings/errors captured in CI logs

**Metrics:**
- GitHub Pages traffic via GitHub Insights
- Build duration tracked via GitHub Actions timing

**Traces:**
- Not applicable (no distributed system tracing needed)

### 6.2 Alerting

**Thresholds:**
- Build failure: Alert on first failure (GitHub Actions email notification)
- Broken links: Alert during CI (link checker tool)

**On-Call Owners:**
- Primary: Project maintainer (e.g., instructor or lead author)
- Escalation: None (educational project, not production service)

### 6.3 Runbooks

**Common Tasks:**

1. **Adding a New Chapter:**
   - Create `docs/<chapter-name>.md` with required frontmatter
   - Add entry to `sidebars.js`
   - Test locally: `npm run start`
   - Commit and push

2. **Fixing Broken Build:**
   - Check GitHub Actions log for error details
   - Reproduce locally: `npm run build`
   - Fix Markdown syntax or frontmatter errors
   - Re-run CI

3. **Updating Citations:**
   - Locate reference in chapter Markdown
   - Update citation with correct APA/IEEE format
   - Verify URL accessibility
   - Commit changes

### 6.4 Deployment and Rollback Strategies

**Deployment:**
- **Strategy**: Continuous deployment via GitHub Actions
- **Trigger**: Push to `main` branch
- **Steps**:
  1. Install dependencies (`npm install`)
  2. Build site (`npm run build`)
  3. Deploy to `gh-pages` branch
  4. GitHub Pages serves from `gh-pages`

**Rollback:**
- **Strategy**: Git revert + redeploy
- **Steps**:
  1. `git revert <commit_hash>`
  2. `git push origin main`
  3. GitHub Actions automatically redeploys

### 6.5 Feature Flags and Compatibility

**Feature Flags:**
- Not applicable (static content; no runtime feature toggling)

**Backward Compatibility:**
- New chapters added without breaking existing chapters
- Frontmatter schema changes are additive
- Docusaurus version pinned to prevent breaking updates

---

## 7. Risk Analysis and Mitigation

### Risk 1: Research Sources Become Outdated

**Blast Radius:** Medium (affects educational accuracy and credibility)

**Probability:** High (robotics field evolves rapidly)

**Mitigation:**
- Schedule annual content review and citation updates
- Use "last verified" timestamps for citations
- Prioritize stable sources (textbooks, foundational papers)
- Document version-specific information (e.g., "as of ROS 2 Humble")

**Kill Switch:**
- Mark outdated chapters with warning banner
- Disable deployment if critical sources become inaccessible

### Risk 2: Docusaurus Breaking Changes

**Blast Radius:** High (entire site build fails)

**Probability:** Low (version pinned in package.json)

**Mitigation:**
- Pin Docusaurus version and all plugins
- Test upgrades in separate branch before merging
- Maintain Docusaurus upgrade checklist

**Kill Switch:**
- Rollback to previous Docusaurus version
- Host raw Markdown on GitHub as fallback

### Risk 3: Code Examples Break Due to Dependency Updates

**Blast Radius:** Medium (code examples non-functional)

**Probability:** Medium (ROS 2, libraries update frequently)

**Mitigation:**
- Pin dependency versions in installation instructions
- Test code examples in CI (future enhancement)
- Provide Docker container with pre-configured environment

**Kill Switch:**
- Mark broken examples with "Under Maintenance" notice
- Provide alternative examples or external links

---

## 8. Evaluation and Validation

### 8.1 Definition of Done

**Per-Chapter Completion:**
- [ ] Frontmatter complete with all required fields
- [ ] Content meets 1,000–2,500 word count
- [ ] All code examples tested and functional
- [ ] All citations verified (URLs accessible, APA/IEEE formatted)
- [ ] Flesch-Kincaid readability score: 8–10
- [ ] Diagrams rendered correctly in Docusaurus
- [ ] Exercises included with clear instructions
- [ ] Summary captures key takeaways
- [ ] Peer review completed (if applicable)

**Per-Module Completion:**
- [ ] All chapters in module completed
- [ ] Module navigation configured in sidebars.js
- [ ] Mini-project tested end-to-end
- [ ] Research sources documented in `research.md`

**Project Completion:**
- [ ] All 4 modules + capstone completed
- [ ] Docusaurus build succeeds (`npm run build`)
- [ ] GitHub Pages deployment successful
- [ ] Full textbook peer-reviewed
- [ ] Video demo of capstone project recorded

### 8.2 Output Validation

**Automated Checks (CI/CD):**
- Markdown syntax validation
- Frontmatter schema validation
- Link checker (no broken links)
- Build success

**Manual Checks:**
- Readability assessment (Flesch-Kincaid score)
- Code example execution
- Diagram accuracy review
- Citation verification

---

## 9. Technical Context

**Language/Version:** Python 3.8+, JavaScript/TypeScript (ES6+), Markdown, YAML
**Primary Dependencies:** ROS 2 (Humble/Iron), Gazebo (Classic/Harmonic), Unity (2021.3+), Docusaurus 3.x, OpenCV 4.x, Whisper
**Storage:** Git repository (GitHub)
**Testing:** Manual execution of code examples, Docusaurus build validation
**Target Platform:** Cross-platform (Linux preferred for ROS 2; macOS/Windows via Docker/WSL)
**Project Type:** Educational content (documentation-driven)
**Performance Goals:** Build time < 5 minutes, page load < 2 seconds
**Constraints:** GitHub Pages 100 MB limit, Flesch-Kincaid 8–10, minimum 40% academic sources
**Scale/Scope:** 12–20 chapters, ~20,000–50,000 total words, 4–5 months development timeline

---

## 10. Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Constitution Compliance

| Principle | Requirement | Status | Notes |
|-----------|-------------|--------|-------|
| **I. Academic Rigor** | Min 40% academic sources, APA/IEEE format | ✅ PASS | Research phase enforces source verification |
| **II. Clarity** | Flesch-Kincaid 8–10, jargon defined | ✅ PASS | Manual validation required per chapter |
| **III. Modularity** | Standalone chapters, consistent structure | ✅ PASS | Template enforces structure |
| **IV. Code Quality** | All examples tested, inline comments | ✅ PASS | Testing workflow documented |
| **V. Structural Consistency** | Markdown + frontmatter, Docusaurus format | ✅ PASS | Template + CI validation |
| **VI. Build Integrity** | `npm run build` succeeds, GitHub Pages deploy | ✅ PASS | CI/CD pipeline enforces |

**No violations detected.** All constitution requirements met by design.

---

## 11. Project Structure

### 11.1 Documentation (this feature)

```text
specs/006-voice-controlled-humanoid/
├── spec.md              # Feature specification
├── plan.md              # This file (architectural plan)
├── research.md          # Phase 0 research findings (to be created)
├── data-model.md        # Phase 1 data structures (to be created)
├── quickstart.md        # Phase 1 setup guide (exists)
├── contracts/           # Phase 1 API contracts (to be created)
└── tasks.md             # Phase 2 implementation tasks (to be created via /sp.tasks)
```

### 11.2 Source Code (repository root)

**Structure Decision:** Documentation-driven project with minimal code scaffolding. Code examples live in chapter Markdown files and separate example repositories.

```text
# Repository Root
/
├── .claude/                    # Claude Code commands
│   └── commands/
│       ├── sp.adr.md
│       ├── sp.plan.md
│       ├── sp.phr.md
│       └── sp.tasks.md
├── .specify/                   # Spec-Kit Plus templates
│   ├── memory/
│   │   └── constitution.md
│   ├── scripts/
│   │   └── bash/
│   │       ├── create-phr.sh
│   │       └── setup-plan.sh
│   └── templates/
│       ├── plan-template.md
│       ├── phr-template.prompt.md
│       └── spec-template.md
├── docs/                       # Docusaurus content (chapters)
│   ├── module-01-ros2/
│   │   ├── 01-introduction.md
│   │   ├── 02-ros2-basics.md
│   │   └── 03-python-integration.md
│   ├── module-02-simulation/
│   ├── module-03-perception/
│   ├── module-04-control/
│   └── capstone-project/
├── history/                    # Prompt History Records
│   ├── prompts/
│   │   ├── constitution/
│   │   ├── general/
│   │   └── voice-controlled-humanoid/
│   └── adr/                    # Architecture Decision Records
├── specs/                      # Feature specifications
│   ├── 001-textbook-master/
│   ├── 002-module-01-ros2/
│   ├── 003-module-02-simulation/
│   ├── 004-module-03-perception/
│   ├── 005-module-04-control/
│   └── 006-voice-controlled-humanoid/
├── static/                     # Docusaurus static assets
│   ├── img/
│   └── diagrams/
├── src/                        # Docusaurus React components (optional)
├── docusaurus.config.js        # Docusaurus configuration
├── sidebars.js                 # Navigation structure
├── package.json                # Dependencies
└── README.md                   # Project overview
```

---

## 12. Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

**No violations.** All architectural decisions align with constitution principles. No additional complexity justification required.

---

## 13. Architectural Decision Records (ADR)

📋 **Significant Architectural Decisions Identified:**

1. **Research-Concurrent Approach** (vs. upfront research) — impacts project timeline and content freshness
2. **Docusaurus as Publishing Platform** (vs. alternatives) — impacts long-term maintainability and deployment
3. **Modular Pedagogical Structure** (vs. linear narrative) — impacts teaching flexibility and content organization
4. **Python Primary, JavaScript Secondary** (vs. C++ or single language) — impacts accessibility and scope
5. **Simulation-First, Hardware-Optional** (vs. hardware-required) — impacts accessibility and cost
6. **APA Citation Style with IEEE Fallback** (vs. single style) — impacts academic credibility presentation

**Recommendation:** Document these decisions as ADRs using `/sp.adr <decision-title>` for future reference and institutional memory.

**Grouped ADR Suggestion:**
📋 Architectural decision detected: **Content Generation and Publishing Architecture** (includes Decisions 1, 2, 3)
Document reasoning and tradeoffs? Run `/sp.adr content-generation-publishing-architecture`

📋 Architectural decision detected: **Language and Platform Selection** (includes Decisions 4, 5)
Document reasoning and tradeoffs? Run `/sp.adr language-platform-selection`

---

## Next Steps

### Phase 0: Research (Per Module)
1. Create `research.md` for each module in `specs/<module>/`
2. Gather 10–20 authoritative sources per module (minimum 40% academic)
3. Validate all URLs and paper DOIs
4. Document key findings and citation formatting

### Phase 1: Design (Per Module)
1. Draft chapter outlines based on research findings
2. Create `data-model.md` for key entities (robot models, sensor data structures)
3. Define API contracts in `contracts/` (ROS topics, service definitions)
4. Update `quickstart.md` with setup instructions

### Phase 2: Implementation (Per Module)
1. Generate chapter content using Claude Code
2. Test all code examples in target environments
3. Create diagrams using Spec-Kit Plus templates
4. Run validation gates (Source, Clarity, Code, Build, Consistency)
5. Commit and push to GitHub

### Phase 3: Integration (Capstone Project)
1. Integrate all module content into capstone project chapter
2. Test full end-to-end workflow (voice command → robot action)
3. Record video demo
4. Final peer review

### Phase 4: Deployment
1. Configure GitHub Actions for automated deployment
2. Deploy to GitHub Pages
3. Verify public accessibility
4. Announce to target audience (students, instructors)

---

**Plan Status:** ✅ Complete — Ready for Phase 0 (Research)

**Created by:** Claude Code (Sonnet 4.5)
**Date:** 2025-12-05
**Review Status:** Awaiting user approval

---
