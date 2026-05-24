# Autumn remote depot

Host `autumn.json` on GitHub Pages or similar, then add it to your global `conductor.pros`:

```json
{
  "depots": {
    "autumn": "https://your-org.github.io/autumn/depot/autumn.json"
  }
}
```

Install or upgrade on a kernel project:

```bash
pros conduct apply autumn
pros conduct upgrade autumn
```

Update the `metadata.location` URL in `autumn.json` when publishing a new release zip (`make template` then upload).
