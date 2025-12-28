<!-- SYNC IMPACT REPORT
Version change: N/A → 1.0.0
List of modified principles: N/A (new constitution)
Added sections: All sections
Removed sections: None
Templates requiring updates:
  - .specify/templates/plan-template.md ✅ updated
  - .specify/templates/spec-template.md ✅ updated
  - .specify/templates/tasks-template.md ✅ updated
  - .specify/templates/adr-template.md ✅ verified
  - .specify/templates/commands/*.md ⚠ pending (directory does not exist)
Follow-up TODOs: None
-->

# AI-Driven Technical Book with Embedded RAG Chatbot Constitution

## Core Principles

### I. Specification-First Development
All features and functionality must be specified using Spec-Kit Plus before implementation. No code should be written without an approved specification that clearly defines requirements, acceptance criteria, and test cases.

### II. Technical Accuracy and Reproducibility
All content and code examples must be technically accurate and fully reproducible. Every implementation detail must be verified to work as described, with clear instructions for users to reproduce results.

### III. Clear, Instructional Writing for Developers
All book content must be written with clear, instructional language that guides developers through concepts and implementations. Technical explanations must be accessible and include practical examples.

### IV. Modular, Maintainable Architecture
System architecture must be modular with clear separation of concerns. Components must be independently testable and maintainable, following clean code principles and established design patterns.

### V. No Hallucinations - Content Grounded in Source Material
The RAG chatbot must only provide answers based on actual content from the book. No generated responses that aren't grounded in the source material. Strict adherence to factual accuracy is required.

### VI. Documentation-Driven Development
Comprehensive documentation must be created and maintained for all features, including API documentation, user guides, and deployment instructions. Documentation is considered part of the deliverable.

## Technology Stack Requirements

- Book written in Docusaurus (MD/MDX) and deployed to GitHub Pages
- All content and features driven by approved specs
- Code clarity, reproducibility, and proper documentation required
- RAG chatbot built with:
  - OpenAI Agents / ChatKit SDKs
  - FastAPI backend
  - Neon Serverless Postgres
  - Qdrant Cloud (Free Tier)
- Chatbot must answer questions about the full book and questions restricted to user-selected text only

## Development Workflow

- Use Spec-Kit Plus and Claude Code for all development
- No paid services beyond stated free tiers
- No hardcoded secrets or undocumented steps
- All implementations must be reproducible by end users
- Continuous integration with automated testing required
- Code reviews must verify compliance with all principles

## Governance

This constitution supersedes all other development practices. All development activities must comply with these principles. Amendments to this constitution require explicit documentation, approval, and migration planning. All pull requests and code reviews must verify constitutional compliance. Complexity must be justified with clear benefits.

**Version**: 1.0.0 | **Ratified**: 2025-12-27 | **Last Amended**: 2025-12-27
