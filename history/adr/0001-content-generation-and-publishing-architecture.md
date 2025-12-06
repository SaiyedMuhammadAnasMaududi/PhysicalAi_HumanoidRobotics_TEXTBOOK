# ADR-0001: Content Generation and Publishing Architecture

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-05
- **Feature:** Physical AI & Humanoid Robotics Textbook
- **Context:** Developing a comprehensive educational textbook for Physical AI and Humanoid Robotics requires selecting an integrated approach for content generation workflow, publishing platform, and pedagogical structure. The textbook will serve students and instructors with varying backgrounds, requiring flexibility while maintaining academic rigor. The project must balance accessibility (free hosting, open-source tools) with professional presentation and long-term maintainability. The rapidly evolving nature of robotics and AI fields necessitates an approach that keeps content current without excessive rework.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

We adopt an integrated content generation and publishing architecture consisting of three tightly-coupled components:

- **Content Generation Workflow:** Research-concurrent methodology (research while writing each module incrementally, not upfront) using Spec-Kit Plus phases (Research → Foundation → Analysis → Synthesis)
- **Publishing Platform:** Docusaurus 3.x static site generator with GitHub Pages hosting, React-based with Markdown content
- **Pedagogical Structure:** Modular design with standalone chapters (self-contained with clear prerequisites), enabling flexible teaching approaches while maintaining curriculum coherence
- **Development Tooling:** Claude Code for AI-assisted content generation, Git for version control, GitHub Actions for CI/CD
- **Content Format:** Markdown with YAML frontmatter, 1,000–2,500 words per chapter, validated against constitution principles

## Consequences

### Positive

- **Content Freshness:** Research-concurrent approach ensures sources stay current in rapidly evolving robotics/AI field; reduces wasted research on out-of-scope topics
- **Zero Cost Hosting:** GitHub Pages provides free, reliable hosting with 99.9% uptime; zero CI/CD costs for public repositories
- **Professional Presentation:** Docusaurus provides documentation-optimized tooling (search, navigation, code highlighting, responsive design) without custom development
- **Flexibility for Educators:** Modular chapters allow instructors to teach topics in varying orders; students can reference specific topics independently
- **Maintainability:** Open-source Docusaurus ensures long-term viability; Markdown content is platform-agnostic and can migrate if needed
- **AI-Assisted Efficiency:** Claude Code accelerates content generation while maintaining consistency through Spec-Kit Plus workflows
- **Git-Based Workflow:** Full version control, audit trail, and collaborative editing via GitHub

### Negative

- **Platform Coupling:** Docusaurus-specific features (React components, plugins) create some migration friction if switching platforms
- **Research Overhead:** Concurrent research adds per-module overhead; may miss cross-module insights that upfront research would reveal
- **Redundancy in Content:** Modular chapters require some concept repetition to maintain standalone comprehension
- **Build Complexity:** Docusaurus requires Node.js ecosystem knowledge; adds build step compared to raw Markdown
- **Limited Offline Access:** GitHub Pages requires internet connection; no native PDF generation (though can be added)
- **Scalability Limits:** GitHub Pages has 100MB build size limit; requires asset optimization for large textbooks

## Alternatives Considered

### Alternative Stack A: Upfront Research + GitBook + Linear Narrative
- **Workflow:** Complete all research before writing, sequential content development
- **Platform:** GitBook hosted platform with proprietary tooling
- **Structure:** Linear narrative requiring sequential reading
- **Why Rejected:** Research becomes stale in fast-moving field; vendor lock-in and limited customization; inflexible for diverse teaching approaches

### Alternative Stack B: Concurrent Research + Custom SSG + Modular Structure
- **Workflow:** Research-concurrent (retained)
- **Platform:** Custom static site generator (Hugo/Jekyll/11ty)
- **Structure:** Modular chapters (retained)
- **Why Rejected:** High maintenance cost for custom tooling; lacks documentation-specific features; no integrated search or navigation

### Alternative Stack C: Concurrent Research + LaTeX/PDF Only + Modular Structure
- **Workflow:** Research-concurrent (retained)
- **Platform:** LaTeX compilation to PDF
- **Structure:** Modular chapters (retained)
- **Why Rejected:** Poor web accessibility; no interactive elements (code blocks, embedded diagrams); limited searchability; difficult collaboration workflow

### Alternative Stack D: Upfront Research + Docusaurus + Tiered Difficulty Structure
- **Workflow:** Upfront comprehensive research
- **Platform:** Docusaurus (retained)
- **Structure:** Beginner/Intermediate/Advanced tracks
- **Why Rejected:** Research staleness risk; complex to maintain consistency across skill levels; harder to coordinate parallel content development

## References

- Feature Spec: [specs/006-voice-controlled-humanoid/spec.md](../../specs/006-voice-controlled-humanoid/spec.md)
- Implementation Plan: [specs/006-voice-controlled-humanoid/plan.md](../../specs/006-voice-controlled-humanoid/plan.md) (sections 2.1, 2.2, 2.3)
- Constitution: [.specify/memory/constitution.md](../../.specify/memory/constitution.md) (Principles III, V, VI)
- Related ADRs: ADR-0002 (Technology Stack for Education)
- Evaluator Evidence: Constitution Check in plan.md (all principles PASS)
