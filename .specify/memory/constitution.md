<!--
Sync Impact Report:
- Version change: [initial] → 1.0.0
- Modified principles: N/A (initial version)
- Added sections: All core principles, content standards, technical requirements, governance
- Removed sections: N/A
- Templates requiring updates:
  ✅ .specify/templates/plan-template.md (constitution check gates added)
  ✅ .specify/templates/spec-template.md (aligned with user story and requirements structure)
  ✅ .specify/templates/tasks-template.md (aligned with user story phases)
- Follow-up TODOs: None
-->

# Physical AI & Humanoid Robotics Textbook Constitution

## Core Principles

### I. Academic Rigor and Source Verification

**Non-Negotiable Rules:**
- All technical definitions MUST come from authoritative sources (research papers, textbooks, IEEE standards, academic lectures)
- Minimum 40% of sources MUST be academically credible (research papers, IEEE/ACM publications, peer-reviewed textbooks)
- Remaining sources MAY include technical blogs, official documentation, or standards bodies
- Zero tolerance for hallucinations or invented references
- All external facts MUST have traceable URLs or paper references
- Citation style MUST follow IEEE format unless otherwise specified

**Rationale:** Academic credibility is foundational for educational materials. Students and instructors must be able to verify and trace every technical claim to established sources.

### II. Clarity and Accessibility

**Non-Negotiable Rules:**
- Reading level MUST target Flesch-Kincaid grade 8–10 (accessible but accurate)
- All jargon MUST be introduced and defined before use
- Simple, accessible language MUST be used while maintaining technical correctness
- Each concept MUST be explained before being referenced in subsequent content
- Standardized callouts MUST be used for notes, tips, and warnings

**Rationale:** The textbook serves beginners and students entering Physical AI and robotics. Clarity ensures comprehension without sacrificing accuracy.

### III. Modularity and Pedagogical Flow

**Non-Negotiable Rules:**
- Each chapter MUST be standalone and independently understandable
- Chapters MUST maintain consistency with the overall curriculum
- Concepts MUST progress from beginner → intermediate → advanced
- Each chapter MUST include:
  - Learning objectives
  - Core content (1,000–2,500 words)
  - Code examples (where applicable)
  - Diagrams (where applicable)
  - Exercises
  - Summary

**Rationale:** Modular design allows flexible teaching approaches while maintaining pedagogical coherence. Students can study chapters in sequence or reference specific topics independently.

### IV. Practical Application and Code Quality

**Non-Negotiable Rules:**
- All code examples MUST run as-is without modification
- Supported languages: JavaScript/TypeScript, Python, MCP/Agent workflows
- Code MUST include inline comments explaining key concepts
- Examples MUST demonstrate real-world robotics concepts and agent development patterns
- Code MUST be tested before inclusion in the textbook

**Rationale:** Hands-on practice is essential for learning Physical AI and robotics. Non-functional examples undermine trust and learning outcomes.

### V. Structural Consistency and Format Standards

**Non-Negotiable Rules:**
- Format MUST be Markdown optimized for Docusaurus
- Frontmatter MUST be included in every chapter file
- Heading hierarchy MUST follow:
  - H1 (#) for chapter titles
  - H2 (##) for major sections
  - H3 (###) for subsections
- Glossary terms MUST be consistent across all chapters
- Diagrams MUST be generated using Spec-Kit Plus templates or compatible tools
- All assets (images, diagrams) MUST be referenced with relative paths

**Rationale:** Consistency ensures professional presentation and reliable build processes. Docusaurus compatibility is required for GitHub Pages deployment.

### VI. Build and Deployment Integrity

**Non-Negotiable Rules:**
- The book MUST compile successfully using `npm run build` without errors
- Build MUST be deployable to GitHub Pages via GitHub Actions
- All chapters MUST pass Spec-Kit validation (accuracy, clarity, sourcing, structure)
- Breaking changes to build or deployment MUST be documented and justified
- All dependencies MUST be versioned in package.json

**Rationale:** A textbook that doesn't build or deploy is unusable. Build integrity ensures the book remains accessible and maintainable.

## Content Standards

### Topical Coverage

The textbook MUST cover:
- Physical AI fundamentals
- Humanoid robotics principles
- Intelligent agents (OpenAI Agents, MCP Tools, multi-agent systems)
- System architecture and design patterns
- Hands-on coding examples with explanations
- Real-world applications and case studies

### Chapter Requirements

Each chapter MUST include:
1. **Frontmatter**: title, description, learning objectives, keywords
2. **Introduction**: context and motivation (100–200 words)
3. **Core Content**: concepts, explanations, examples (800–2,100 words)
4. **Code Examples**: at least one working example (where applicable)
5. **Diagrams**: visual explanations of complex concepts (where applicable)
6. **Exercises**: 3–5 exercises for student practice
7. **Summary**: key takeaways (100–200 words)
8. **References**: IEEE-formatted citations for all sources

### Scope and Scale

- Total book size: Minimum 12–20 chapters
- Each chapter: 1,000–2,500 words (excluding code and diagrams)
- Chapter count MAY expand based on curriculum needs
- Cross-references between chapters MUST use consistent linking format

## Technical Requirements

### Development Workflow

All chapter creation and revision MUST use:
- Claude Code for drafting, refinement, and verification
- Spec-Kit Plus workflows for planning and validation
- Git for version control with meaningful commit messages
- Feature branches for new chapters or major revisions

### Validation Gates

Before marking any chapter complete, it MUST pass:
1. **Source Check**: All claims verified with cited sources
2. **Clarity Check**: Reading level verified (Flesch-Kincaid 8–10)
3. **Code Check**: All examples tested and functional
4. **Build Check**: Chapter integrates without build errors
5. **Consistency Check**: Formatting, terminology, and structure aligned with constitution

### Quality Assurance

- Peer/mentor review MUST indicate correctness and pedagogical clarity
- Factual accuracy MUST be verified against authoritative sources
- Code examples MUST be executed in target environments before publication
- Diagrams MUST accurately represent described concepts

## Governance

### Amendment Procedure

1. Proposed amendments MUST be documented with rationale
2. Impact on existing chapters MUST be assessed
3. Version number MUST be incremented per semantic versioning:
   - **MAJOR**: Backward-incompatible changes (e.g., removing a core principle)
   - **MINOR**: New principles or sections added
   - **PATCH**: Clarifications, wording improvements, non-semantic fixes
4. Amendments MUST be reviewed before adoption
5. Migration plan MUST be provided for changes affecting existing content

### Versioning Policy

- Constitution version MUST be tracked in this file
- Changes MUST be documented in Sync Impact Report (HTML comment at top)
- Breaking changes MUST be clearly marked and justified

### Compliance Review

- All chapters MUST verify compliance with current constitution version
- Non-compliance MUST be documented as technical debt with remediation plan
- Complexity or exceptions MUST be justified in writing

### Development Guidance

- Runtime development guidance lives in `CLAUDE.md`
- Spec-Kit Plus templates live in `.specify/templates/`
- Chapter specifications live in `specs/<chapter-name>/`
- Prompt History Records live in `history/prompts/`
- Architecture Decision Records live in `history/adr/`

**Version**: 1.0.0 | **Ratified**: 2025-12-05 | **Last Amended**: 2025-12-05
