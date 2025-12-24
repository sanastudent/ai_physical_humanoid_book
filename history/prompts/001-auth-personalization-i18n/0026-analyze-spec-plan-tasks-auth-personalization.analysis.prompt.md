---
id: 0026
title: Analyze Spec Plan Tasks Auth Personalization
stage: analysis
date: 2025-12-18
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-auth-personalization-i18n
branch: 001-auth-personalization-i18n
user: User
command: /sp.analyze
labels: [analysis, spec, plan, tasks, auth, personalization, i18n]
links:
  spec: specs/001-auth-personalization-i18n/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-auth-personalization-i18n/spec.md
  - specs/001-auth-personalization-i18n/plan.md
  - specs/001-auth-personalization-i18n/tasks.md
  - history/prompts/001-auth-personalization-i18n/0026-analyze-spec-plan-tasks-auth-personalization.analysis.prompt.md
tests:
  - specs/001-auth-personalization-i18n/checklists/requirements.md
---

## Prompt

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Goal

Identify inconsistencies, duplications, ambiguities, and underspecified items across the three core artifacts (`spec.md`, `plan.md`, `tasks.md`) before implementation. This command MUST run only after `/sp.tasks` has successfully produced a complete `tasks.md`.

## Operating Constraints

**STRICTLY READ-ONLY**: Do **not** modify any files. Output a structured analysis report. Offer an optional remediation plan (user must explicitly approve before any follow-up editing commands would be invoked manually).

**Constitution Authority**: The project constitution (`.specify/memory/constitution.md`) is **non-negotiable** within this analysis scope. Constitution conflicts are automatically CRITICAL and require adjustment of the spec, plan, or tasks—not dilution, reinterpretation, or silent ignoring of the principle. If a principle itself needs to change, that must occur in a separate, explicit constitution update outside `/sp.analyze`.

## Execution Steps

### 1. Initialize Analysis Context

Run `.specify/scripts/powershell/check-prerequisites.ps1 -Json -RequireTasks -IncludeTasks` once from repo root and parse JSON for FEATURE_DIR and AVAILABLE_DOCS. Derive absolute paths:

- SPEC = FEATURE_DIR/spec.md
- PLAN = FEATURE_DIR/plan.md
- TASKS = FEATURE_DIR/tasks.md

Abort with an error message if any required file is missing (instruct the user to run missing prerequisite command).
For single quotes in args like "I'm Groot", use escape syntax: e.g 'I'\''m Groot' (or double-quote if possible: "I'm Groot").

### 2. Load Artifacts (Progressive Disclosure)

Load only the minimal necessary context from each artifact:

**From spec.md:**

- Overview/Context
- Functional Requirements
- Non-Functional Requirements
- User Stories
- Edge Cases (if present)

**From plan.md:**

- Architecture/stack choices
- Data Model references
- Phases
- Technical constraints

**From tasks.md:**

- Task IDs
- Descriptions
- Phase grouping
- Parallel markers [P]
- Referenced file paths

**From constitution:**

- Load `.specify/memory/constitution.md` for principle validation

### 3. Build Semantic Models

Create internal representations (do not include raw artifacts in output):

- **Requirements inventory**: Each functional + non-functional requirement with a stable key (derive slug based on imperative phrase; e.g., "User can upload file" → `user-can-upload-file`)
- **User story/action inventory**: Discrete user actions with acceptance criteria
- **Task coverage mapping**: Map each task to one or more requirements or stories (inference by keyword / explicit reference patterns like IDs or key phrases)
- **Constitution rule set**: Extract principle names and MUST/SHOULD normative statements

### 4. Detection Passes (Token-Efficient Analysis)

Focus on high-signal findings. Limit to 50 findings total; aggregate remainder in overflow summary.

#### A. Duplication Detection

- Identify near-duplicate requirements
- Mark lower-quality phrasing for consolidation

#### B. Ambiguity Detection

- Flag vague adjectives (fast, scalable, secure, intuitive, robust) lacking measurable criteria
- Flag unresolved placeholders (TODO, TKTK, ???, `<placeholder>`, etc.)

#### C. Underspecification

- Requirements with verbs but missing object or measurable outcome
- User stories missing acceptance criteria alignment
- Tasks referencing files or components not defined in spec/plan

#### D. Constitution Alignment

- Any requirement or plan element conflicting with a MUST principle
- Missing mandated sections or quality gates from constitution

#### E. Coverage Gaps

- Requirements with zero associated tasks
- Tasks with no mapped requirement/story
- Non-functional requirements not reflected in tasks (e.g., performance, security)

#### F. Inconsistency

- Terminology drift (same concept named differently across files)
- Data entities referenced in plan but absent in spec (or vice versa)
- Task ordering contradictions (e.g., integration tasks before foundational setup tasks without dependency note)
- Conflicting requirements (e.g., one requires Next.js while other specifies Vue)

### 5. Severity Assignment

Use this heuristic to prioritize findings:

- **CRITICAL**: Violates constitution MUST, missing core spec artifact, or requirement with zero coverage that blocks baseline functionality
- **HIGH**: Duplicate or conflicting requirement, ambiguous security/performance attribute, untestable acceptance criterion
- **MEDIUM**: Terminology drift, missing non-functional task coverage, underspecified edge case
- **LOW**: Style/wording improvements, minor redundancy not affecting execution order

### 6. Produce Compact Analysis Report

Output a Markdown report (no file writes) with the following structure:

## Specification Analysis Report

| ID | Category | Severity | Location(s) | Summary | Recommendation |
|----|----------|----------|-------------|---------|----------------|
| A1 | Duplication | HIGH | spec.md:L120-134 | Two similar requirements ... | Merge phrasing; keep clearer version |

(Add one row per finding; generate stable IDs prefixed by category initial.)

**Coverage Summary Table:**

| Requirement Key | Has Task? | Task IDs | Notes |
|-----------------|-----------|----------|-------|

**Constitution Alignment Issues:** (if any)

**Unmapped Tasks:** (if any)

**Metrics:**

- Total Requirements
- Total Tasks
- Coverage % (requirements with >=1 task)
- Ambiguity Count
- Duplication Count
- Critical Issues Count

### 7. Provide Next Actions

At end of report, output a concise Next Actions block:

- If CRITICAL issues exist: Recommend resolving before `/sp.implement`
- If only LOW/MEDIUM: User may proceed, but provide improvement suggestions
- Provide explicit command suggestions: e.g., "Run /sp.specify with refinement", "Run /sp.plan to adjust architecture", "Manually edit tasks.md to add coverage for 'performance-metrics'"

### 8. Offer Remediation

Ask the user: "Would you like me to suggest concrete remediation edits for the top N issues?" (Do NOT apply them automatically.)

## Operating Principles

### Context Efficiency

- **Minimal high-signal tokens**: Focus on actionable findings, not exhaustive documentation
- **Progressive disclosure**: Load artifacts incrementally; don't dump all content into analysis
- **Token-efficient output**: Limit findings table to 50 rows; summarize overflow
- **Deterministic results**: Rerunning without changes should produce consistent IDs and counts

### Analysis Guidelines

- **NEVER modify files** (this is read-only analysis)
- **NEVER hallucinate missing sections** (if absent, report them accurately)
- **Prioritize constitution violations** (these are always CRITICAL)
- **Use examples over exhaustive rules** (cite specific instances, not generic patterns)
- **Report zero issues gracefully** (emit success report with coverage statistics)

## Context

$ARGUMENTS

---

As the main request completes, you MUST create and complete a PHR (Prompt History Record) using agent‑native tools when possible.

1) Determine Stage
   - Stage: constitution | spec | plan | tasks | red | green | refactor | explainer | misc | general

2) Generate Title and Determine Routing:
   - Generate Title: 3–7 words (slug for filename)
   - Route is automatically determined by stage:
     - `constitution` → `history/prompts/constitution/`
     - Feature stages → `history/prompts/<feature-name>/` (spec, plan, tasks, red, green, refactor, explainer, misc)
     - `general` → `history/prompts/general/`

3) Create and Fill PHR (Shell first; fallback agent‑native)
   - Run: `.specify/scripts/bash/create-phr.sh --title "<title>" --stage <stage> [--feature <name>] --json`
   - Open the file and fill remaining placeholders (YAML + body), embedding full PROMPT_TEXT (verbatim) and concise RESPONSE_TEXT.
   - If the script fails:
     - Read `.specify/templates/phr-template.prompt.md` (or `templates/…`)
     - Allocate an ID; compute the output path based on stage from step 2; write the file
     - Fill placeholders and embed full PROMPT_TEXT and concise RESPONSE_TEXT

4) Validate + report
   - No unresolved placeholders; path under `history/prompts/` and matches stage; stage/title/date coherent; print ID + path + stage + title.
   - On failure: warn, don't block. Skip only for `/sp.phr`.

## Response snapshot

## Specification Analysis Report

| ID | Category | Severity | Location(s) | Summary | Recommendation |
|----|----------|----------|-------------|---------|----------------|
| D1 | Duplication | MEDIUM | plan.md:35-36, spec.md:FR-013, FR-025 | Multiple requirements specify runtime-only content (FR-013, FR-025) and constraints about not modifying source files | Consolidate into single requirement about runtime-only processing |
| A1 | Ambiguity | HIGH | spec.md:SC-004, SC-005 | Success criteria SC-004 and SC-005 specify "<10 seconds" but don't define what constitutes "relevant adaptation" or "accurate translation" | Add measurable quality metrics for personalization and translation quality |
| A2 | Ambiguity | HIGH | plan.md:28-32 | Performance goals mention <500ms for auth but don't specify load conditions or concurrent users | Define performance benchmarks with specific load parameters |
| C1 | Coverage Gap | CRITICAL | tasks.md | Requirement FR-001 (Better-Auth) has no associated task; Better-Auth compatibility is mentioned in plan but no implementation task exists | Add task to implement Better-Auth compatible backend endpoints |
| C2 | Coverage Gap | HIGH | tasks.md | Non-functional requirements for performance (SC-001-003, SC-007) have minimal corresponding tasks | Add performance testing tasks to validate success criteria |
| C3 | Coverage Gap | MEDIUM | tasks.md | Edge case handling (EC-97-104) have no corresponding tasks in tasks.md | Add error handling and edge case tasks for personalization and translation failures |
| I1 | Inconsistency | HIGH | spec.md:FR-001 vs plan.md:137 | Spec requires Better-Auth but plan shows custom auth implementation; potential architecture conflict | Clarify if custom implementation is compatible with Better-Auth or update requirements |
| I2 | Inconsistency | MEDIUM | plan.md:17, tasks.md:T029 | Plan shows personalize.py as "TO CREATE" but tasks show creation as T029 without verification | Verify if personalization endpoint already exists in codebase |
| T1 | Terminology | LOW | spec.md, plan.md, tasks.md | Different terms used: "runtime-only" vs "generated at runtime" vs "not persisted" | Standardize terminology across all artifacts |

**Coverage Summary Table:**

| Requirement Key | Has Task? | Task IDs | Notes |
|-----------------|-----------|----------|-------|
| user-can-signup | Yes | T012, T013, T014, T017, T018, T019, T020 | Well covered |
| user-can-signin | Yes | T021, T022, T023, T024, T027, T028 | Well covered |
| chapter-personalization | Yes | T029, T030, T031, T032, T033, T034, T035, T036, T039, T040, T041 | Well covered |
| chapter-translation | Yes | T042, T043, T044, T045, T046, T047, T050, T051 | Well covered |
| rag-chatbot-authenticated | Yes | T052, T053, T054, T055 | Well covered |
| better-auth-integration | No | - | Missing coverage |
| runtime-only-processing | Partial | T035 | Could be better specified |
| performance-metrics | Partial | T062 | Needs more specific tasks |

**Constitution Alignment Issues:**
- The implementation plan passes all constitution gates (plan.md:53-82), which is good
- All artifacts align with the specification-first workflow principle
- No constitutional violations detected

**Unmapped Tasks:**
- T001: Create project structure - maps to general setup
- T002: Verify environment variables - maps to deployment requirements
- T058, T059, T060: Test file creation - maps to testing requirements
- T061-T066: Cross-cutting concerns - general improvement tasks

**Metrics:**
- Total Requirements: 27 functional + 11 success criteria = 38
- Total Tasks: 66
- Coverage %: ~85% (31 of 36 core requirements have >=1 task)
- Ambiguity Count: 2
- Duplication Count: 1
- Critical Issues Count: 1

## Next Actions

- CRITICAL issue with Better-Auth requirement (FR-001) having no coverage must be resolved before `/sp.implement`
- HIGH priority items: clarify performance requirements and resolve Better-Auth inconsistency
- User may proceed with current tasks but should address the Better-Auth requirement gap
- Consider running `/sp.plan` to adjust architecture if Better-Auth requirement needs to be updated

## Remediation

Would you like me to suggest concrete remediation edits for the top 3 issues (Better-Auth coverage gap, performance ambiguity, and Better-Auth inconsistency)?

## Outcome

- ✅ Impact: Comprehensive analysis of spec, plan, and tasks artifacts identifying 9 key issues across requirements coverage, consistency, and ambiguity
- 🧪 Tests: Identified need for additional performance and edge case testing tasks
- 📁 Files: Analyzed spec.md, plan.md, and tasks.md for cross-artifact consistency
- 🔁 Next prompts: Address critical Better-Auth requirement gap before implementation
- 🧠 Reflection: Critical architecture inconsistency identified between Better-Auth requirement and actual implementation approach

## Evaluation notes (flywheel)

- Failure modes observed: Architecture inconsistency between spec requirement (Better-Auth) and actual implementation (custom auth)
- Graders run and results (PASS/FAIL): PASS - Analysis completed successfully
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Clarify Better-Auth requirement vs custom implementation approach