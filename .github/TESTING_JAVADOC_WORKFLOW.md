# 🚀 Quick Start: Testing the Javadoc Workflow

## The Fastest Way to Test This PR

**You can test this workflow RIGHT NOW without merging!**

### Step-by-Step Instructions:

#### 1. Navigate to the Workflow
Click this link: [**Generate and Publish Javadoc Workflow**](https://github.com/WHS-FRC-3467/Skip-5.16/actions/workflows/javadoc.yml)

#### 2. Find the "Run workflow" Button
Look for a **blue button** labeled "Run workflow" on the right side of the page, above the list of workflow runs.

```
┌─────────────────────────────────────────────────────────────┐
│ Actions > Generate and Publish Javadoc                      │
├─────────────────────────────────────────────────────────────┤
│                                     ┌──────────────────┐     │
│ This workflow has a workflow_dispatch event trigger.   │     │
│                                     │ Run workflow ▼   │◄────┐
│                                     └──────────────────┘     │
└─────────────────────────────────────────────────────────────┘
                                                                │
                                                   Click This! ─┘
```

#### 3. Select Your Branch
When you click "Run workflow", a dropdown appears:
- **Branch:** Select `copilot/fix-javadocs-generation-errors`
- Click the green **"Run workflow"** button in the dropdown

#### 4. Watch It Run
- The workflow will start immediately
- You can click on the run to see live logs
- It takes about 2-5 minutes to complete

#### 5. View the Results
Once successful, you can:
- ✅ See the green checkmark on the workflow run
- 📦 Download the Javadoc artifact from the run
- 🌐 Visit the published site: https://whs-frc-3467.github.io/Skip-5.16/

---

## Prerequisites ⚙️

**For GitHub Pages deployment to work**, a repository admin must:

1. Go to **Settings** → **Pages**
2. Under "Source", select **"GitHub Actions"** (not "Deploy from branch")
3. Save

This only needs to be done once.

---

## Alternative: Using GitHub CLI

If you have the GitHub CLI installed:

```bash
# Trigger the workflow
gh workflow run javadoc.yml --ref copilot/fix-javadocs-generation-errors

# Watch it run
gh run watch

# List recent runs
gh run list --workflow=javadoc.yml --limit 5
```

---

## Troubleshooting 🔧

**Q: I don't see the "Run workflow" button**
- A: You need push/write access to the repository

**Q: The workflow runs but deployment fails**
- A: Check that GitHub Pages is enabled (see Prerequisites above)

**Q: I see "Environment protection rules" error**
- A: The first deployment may require admin approval in Settings → Environments

---

## What This Workflow Does

1. ✓ Checks out the code from your selected branch
2. ✓ Sets up Java 17
3. ✓ Runs `./gradlew javadoc` to generate documentation (now includes annotation processor via delombok)
4. ✓ Uploads the Javadoc as a build artifact
5. ✓ Deploys the Javadoc to GitHub Pages

All done automatically! 🎉

## Recent Fixes (This PR)

This PR fixes multiple javadoc generation errors that were causing the workflow to fail:

- **Fixed HTML entities**: Escaped ampersands (`&` → `&amp;`) throughout javadoc comments
- **Fixed malformed HTML**: Escaped less-than operators (`<=` → `&lt;=`)  
- **Fixed invalid references**: Removed broken `{@link}` tags to non-existent classes and methods
- **Fixed @param mismatches**: Corrected javadoc parameter names to match method signatures
- **Enhanced build config**: Configured javadoc task to use delombok source (includes AdvantageKit's AutoLogged classes)
- **Optimized workflow**: Removed redundant compilation step
