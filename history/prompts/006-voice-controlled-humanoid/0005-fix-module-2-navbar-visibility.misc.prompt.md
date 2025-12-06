---
id: 0005
title: Fix Module 2 Navbar Visibility
stage: misc
date: 2025-12-06
surface: agent
model: claude-sonnet-4-5-20250929
feature: 006-voice-controlled-humanoid
branch: main
user: developer
command: /sp.implement
labels: ["bugfix", "navbar", "module-2", "docusaurus"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - docusaurus.config.ts
tests:
  - npm run build (successful)
---

## Prompt

i have done it but still only the module is visible

## Response snapshot

The issue was that Module 2 was configured in `sidebars.ts` but not added to the navbar in `docusaurus.config.ts`. The navbar is the top navigation menu that users click to access different sections.

Fixed by adding Module 2 to two places in `docusaurus.config.ts`:
1. Navbar items (line 58-63) - Added "Module 2: Simulation" navigation link
2. Footer links (line 81-84) - Added Module 2 footer link

Build verified successfully with both server and client compilation passing.

## Outcome

- ✅ Impact: Module 2 now fully visible and accessible in the website navigation
- 🧪 Tests: Build test passed (Server: 2.42m, Client: 3.70m)
- 📁 Files: Modified docusaurus.config.ts to add navbar and footer configuration for Module 2
- 🔁 Next prompts: Commit and push both sidebars.ts and docusaurus.config.ts to deploy
- 🧠 Reflection: Previous implementation only updated the sidebar definition but missed the navbar configuration, which is required for top-level navigation in Docusaurus. The navbar controls which sidebars are accessible from the main menu.

## Evaluation notes (flywheel)

- Failure modes observed: Incomplete configuration - sidebar defined but not exposed in navbar
- Graders run and results (PASS/FAIL): Build test PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Verify navbar behavior on live site after deployment
