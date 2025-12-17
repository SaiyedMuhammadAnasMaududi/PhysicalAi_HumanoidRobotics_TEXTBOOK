<!--
Sync Impact Report:
- Version change: 1.0.0 → 2.0.0
- Modified principles: All principles updated to reflect RAG chatbot project requirements (Academic Rigor → Accuracy, Modularity → Traceability, Practical Application → Reliability, etc.)
- Added sections: Chatbot and RAG Requirements section, RAG Pipeline and Infrastructure section, Accuracy Check and RAG Pipeline Check validation gates
- Removed sections: Original chapter requirements (replaced with chatbot-specific requirements)
- Templates requiring updates:
  ⚠ .specify/templates/plan-template.md (needs alignment with new principles)
  ⚠ .specify/templates/spec-template.md (needs alignment with new requirements)
  ⚠ .specify/templates/tasks-template.md (needs alignment with new validation gates)
- Follow-up TODOs: Update templates to align with new constitution
-->

# Physical AI & Humanoid Robotics Textbook Constitution

## Core Principles

### I. Accuracy and Source Verification

**Non-Negotiable Rules:**
- Responses MUST reflect the content of the book; no hallucinations allowed
- All responses MUST be verified against selected text from the book content
- Zero tolerance for fabricated or invented references
- Information MUST be traceable to specific chapters or sections of the book
- Citation format MUST follow APA style for content taken from the book
- When content is unavailable, chatbot MUST explicitly indicate this condition

**Rationale:** Accuracy is foundational for educational materials. Students and instructors must be able to verify and trust that all responses are grounded in the book's content.

### II. Clarity and Accessibility

**Non-Negotiable Rules:**
- Chatbot answers MUST be concise, clear, and understandable for learners of Physical AI and Humanoid Robotics
- All jargon MUST be explained in context appropriate for the target audience
- Simple, accessible language MUST be used while maintaining technical correctness
- Responses MUST be tailored to the comprehension level of robotics learners
- Standardized callouts SHOULD be used for notes, tips, and warnings in responses

**Rationale:** The chatbot serves beginners and students entering Physical AI and robotics. Clarity ensures comprehension without sacrificing accuracy.

### III. Traceability and Reproducibility

**Non-Negotiable Rules:**
- All information MUST be traceable to the book's content with specific chapter/section references
- Retrieval pipeline MUST use embeddings stored in Qdrant (cloud free-tier) and Neon Serverless Postgres
- Chatbot setup and embeddings pipeline MUST be reproducible using provided code and instructions
- Source documents for embeddings MUST be versioned and tracked
- Retrieval results MUST include provenance information linking to original content

**Rationale:** Educational integrity requires that students can verify information and reproduce the system. Traceability ensures accountability and trust.

### IV. Reliability and Performance

**Non-Negotiable Rules:**
- System MUST handle concurrent users (≥ 10 simultaneous users)
- Average response latency MUST be ≤ 3 seconds for retrieval + generation pipeline
- System MUST provide fallback/error handling for all failure modes
- Chatbot MUST maintain consistent availability during operation
- Performance metrics MUST be monitored and logged for optimization

**Rationale:** A reliable system is essential for educational use. Students need consistent, fast responses to maintain engagement and learning flow.

### V. Extensibility and Technology Standards

**Non-Negotiable Rules:**
- Tech stack MUST use OpenAI Agents SDK / ChatKit SDK, FastAPI backend, Neon Postgres, Qdrant Cloud free-tier
- Model configuration MUST support Gemini or Litellm for reasoning and response generation
- Architecture SHOULD support upgrading models or adding new knowledge sources without breaking existing functionality
- Agent configuration MUST be clearly defined with prompt templates and reasoning steps
- Data scope MUST be limited to book content or user-selected text for responses

**Rationale:** The system needs to evolve with advancing technology while maintaining compatibility. Clear technology standards ensure maintainability and scalability.

### VI. Security and Ethical Standards

**Non-Negotiable Rules:**
- User input MUST be sanitized to prevent injection attacks
- No sensitive user data MAY be logged or stored
- Chatbot interaction MUST follow ethical AI guidelines (no generation of harmful or biased content)
- Deployment MUST be embedded within the Docusaurus book deployed on GitHub Pages
- All interactions SHOULD respect user privacy and data protection principles

**Rationale:** Security and ethical considerations are paramount in educational technology. Users must trust that their interactions are safe and appropriate.

## Content Standards

### Topical Coverage

The textbook and chatbot knowledge base MUST cover:
- Physical AI fundamentals
- Humanoid robotics principles
- RAG (Retrieval-Augmented Generation) systems
- Intelligent agents (OpenAI Agents, MCP Tools, multi-agent systems)
- System architecture and design patterns
- Hands-on coding examples with explanations
- Real-world applications and case studies

### Chatbot and RAG Requirements

The integrated RAG chatbot MUST:
1. **Accurately respond** to questions based on book content with ≥ 95% accuracy on test queries
2. **Index all book content sections** in the retrieval pipeline
3. **Provide precise and contextually relevant** answers to questions based on highlighted text
4. **Include provenance information** with each response citing specific book sections
5. **Support user-selected text queries** for focused, contextual responses
6. **Maintain auditable reasoning steps** for debugging and educational purposes

### Scope and Scale

- Total book size: Minimum 12–20 chapters
- Each chapter: 1,000–2,500 words (excluding code and diagrams)
- Chapter count MAY expand based on curriculum needs
- Cross-references between chapters MUST use consistent linking format
- Embedded chatbot interface MUST be seamlessly integrated within Docusaurus documentation

## Technical Requirements

### Development Workflow

All chapter creation, RAG pipeline development, and chatbot implementation MUST use:
- Claude Code for drafting, refinement, and verification
- Spec-Kit Plus workflows for planning and validation
- Git for version control with meaningful commit messages
- Feature branches for new chapters or major revisions
- Reproducible deployment instructions included in the book repository

### RAG Pipeline and Infrastructure

The retrieval-augmented generation system MUST:
- Use Qdrant (cloud free-tier) for vector embeddings storage
- Use Neon Serverless Postgres for metadata and structured data
- Support both Gemini and Litellm models for reasoning and response generation
- Include fully reproducible setup and deployment instructions
- Handle concurrent users (≥ 10 simultaneous users) with appropriate performance
- Implement proper error handling and fallback mechanisms

### Validation Gates

Before marking any chapter or chatbot feature complete, it MUST pass:
1. **Source Check**: All claims verified with cited sources from book content
2. **Accuracy Check**: Chatbot responses grounded in book content for ≥ 95% of test queries
3. **RAG Pipeline Check**: All book content sections correctly indexed in retrieval system
4. **Performance Check**: Response latency ≤ 3 seconds for retrieval + generation pipeline
5. **Build Check**: Chapter and chatbot integrate without build errors
6. **Security Check**: User input sanitized and no sensitive data logged
7. **Consistency Check**: Formatting, terminology, and structure aligned with constitution

### Quality Assurance

- Peer/mentor review MUST indicate correctness and pedagogical clarity
- Factual accuracy MUST be verified against authoritative sources
- Code examples and RAG pipeline MUST be executed in target environments before publication
- Chatbot responses MUST be auditable with internal reasoning steps logged for debugging
- System MUST pass concurrency and stress tests (≥ 10 simultaneous users)

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

**Version**: 2.0.0 | **Ratified**: 2025-12-05 | **Last Amended**: 2025-12-16
