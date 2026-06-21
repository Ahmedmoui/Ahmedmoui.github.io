# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Ahmed Moussaoui's personal portfolio site (`ahmedmoui.github.io`) — a **Jekyll** site served by **GitHub Pages**. It started from the "freeToEngineer" / Lowinertia template (see `README.md`, which is the original template's end-user setup guide), then was customized. There is no application code or test suite; the work here is content (Markdown/YAML/front matter) and presentation (Liquid templates + hand-written CSS).

## Commands

```bash
bundle install                 # install gems (first time / after Gemfile change)
bundle exec jekyll serve       # local dev server with live reload at http://localhost:4000
bundle exec jekyll build       # one-off build into _site/
```

There is no lint or test step. The `wdm` gem provides file-watching on Windows.

## Deployment

- **The site** deploys automatically: GitHub Pages runs its own Jekyll build whenever you push to `main`. There is no site-build workflow in `.github/workflows/` — don't add one expecting it to deploy; GitHub Pages does that natively. The `github-pages` gem (not a pinned `jekyll` gem) keeps the local build matched to GH Pages' supported versions.
- **The resume** is the one build step we own: `.github/workflows/build-resume.yml` compiles `assets/resume/resume.tex` → `assets/resume/resume.pdf` (via `xu-cheng/latex-action`) and auto-commits the PDF with a `[skip ci]` message. It triggers **only** on changes to the `.tex` or the workflow itself, so the bot's PDF commit can't loop. **Edit `resume.tex`, never hand-edit `resume.pdf`** — your PDF edits will be overwritten on the next `.tex` change.

## Content architecture

Most "what the site says" lives in data/config, not templates. To change copy or settings, prefer these over editing HTML:

- **`_config.yml`** — the main content + config surface: hero name/headline/description, the **Skills** section list, external-link slugs, Formspree key, profile image path, and the **active theme colors** (the `colors:` block). The other named theme blocks (`basic`/`mint`/`lemon`/`pink`) lower in the file are copy-paste examples, not active — and several have inconsistent indentation, so copy carefully into `colors:`. Changing `_config.yml` requires restarting `jekyll serve`.
- **`_data/resume.yml`** — single source of truth for the home-page **terminal widget** (`whoami`, `cat skills.txt`) and the **availability status** (the hero pill + the terminal's status line). Set `status.available: false` to hide both at once.

## Page assembly (Liquid)

- **`_layouts/wrapper.html`** is the base shell every page uses: `<head>`, fonts, Font Awesome, OG/Twitter meta (built from `site.url` + `page.url` + `site.profile_image`), navbar, footer, and the `script.js` include. New top-level pages should declare `layout: wrapper`.
- **`_layouts/post.html`** renders an individual project page (image + title + description + skills card + body) and itself wraps in `wrapper`.
- **`index.html`** is just an assembly of includes: `hero` → `terminal` → `projects` → `skills` → `contact`. The reusable section includes live in `_includes/` (`hero.html`, `terminal.html`, `projects.html`, `skills.html`, `contact.html`, `navbar.html`, `footer.html`).
- **`assets/js/script.js`** is all the front-end behavior: mobile nav toggle, sticky-nav shadow, IntersectionObserver scroll-reveal (`.reveal-enabled` is set inline in `<head>` so content is never permanently hidden if JS fails), and the terminal type-out animation.

## Projects collection

Projects are a Jekyll collection (`_config.yml`: `output: true`, `permalink: /projects/:path/`), sorted by the `order` front-matter field on the home/projects grid. To add a project:

1. Create `_projects/<project_name>/index.md`.
2. Front matter must include: `layout: post`, `order:` (sort position), `title:`, `description:`, `skills:` (list), and `main-image:` (a **leading-slash filename**, e.g. `/arm.jpg`, that gets appended to the project's folder path by `projects.html` / `post.html`).
3. Put images in the same folder. Reference them from the Markdown body with an **absolute path**: `/_projects/<project_name>/<image>` (this is how the existing projects do it).

Reusable content includes available inside project Markdown: `image-gallery.html` (comma-separated `images=`, optional `height=`; resolves paths relative to the page URL) and `youtube-video.html`.

## Things that look like content but aren't

- `_site/` and `css/_site/` are **build output** (gitignored / stray) — never edit by hand.
- `project_examples/` and `Reference/template.md` are leftover template scaffolding/examples, not part of the live site.
- `.claude/` is local tooling (gitignored) and includes design mockups (`mockup.html`, `terminal-mock.html`) used while iterating, not shipped assets.
