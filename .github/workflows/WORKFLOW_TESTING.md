# Testing GitHub Actions Workflows

## How to Manually Trigger the Javadoc Workflow

The Javadoc generation workflow can be manually triggered to test it before merging changes.

### Method 1: GitHub Web Interface

1. Navigate to the repository's [Actions tab](https://github.com/WHS-FRC-3467/Skip-5.16/actions)
2. Click on "Generate and Publish Javadoc" in the workflows list
3. Click the **"Run workflow"** dropdown button (top right, above the workflow runs list)
4. Select the branch you want to test from (e.g., your PR branch)
5. Click the **"Run workflow"** button to start the workflow

### Method 2: GitHub CLI

If you have the GitHub CLI installed:

```bash
# Run on a specific branch
gh workflow run javadoc.yml --ref copilot/add-javadoc-generation-workflow

# Check the workflow status
gh run list --workflow=javadoc.yml

# View workflow logs
gh run view --workflow=javadoc.yml
```

### Method 3: REST API

Using curl:

```bash
curl -X POST \
  -H "Accept: application/vnd.github+json" \
  -H "Authorization: Bearer YOUR_TOKEN" \
  https://api.github.com/repos/WHS-FRC-3467/Skip-5.16/actions/workflows/javadoc.yml/dispatches \
  -d '{"ref":"copilot/add-javadoc-generation-workflow"}'
```

## Viewing Results

After the workflow runs successfully:

1. **Workflow Artifacts**: Download the generated Javadoc from the workflow run's artifacts section
2. **GitHub Pages**: If deployment succeeds, view the published docs at: https://whs-frc-3467.github.io/Skip-5.16/

## Prerequisites for GitHub Pages Deployment

For the workflow to successfully publish to GitHub Pages:

1. Go to **Repository Settings** → **Pages**
2. Under "Source", select **"GitHub Actions"**
3. Save the settings

This only needs to be done once by a repository administrator.

## Troubleshooting

- **"Run workflow" button not visible**: You must have push access to the repository
- **Deployment fails**: Check that GitHub Pages is enabled with "GitHub Actions" as the source
- **Permission errors**: Ensure the workflow has the correct permissions (already configured)
