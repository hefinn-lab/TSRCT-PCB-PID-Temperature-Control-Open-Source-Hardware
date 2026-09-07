# Applying this release metadata package

Copy the files from this package into the repository root, preserving any existing third-party notices. Commit them before creating the GitHub release.

- Tag: `v1.1`
- Release title: `TSRCT-PCB-01 v1.1 — Initial public release`
- Release description: paste the body of RELEASE-NOTES.md (the GitHub title field already contains its heading).
- Citation filename: **CITATION.cff** exactly, with the lowercase extension.

CITATION.cff uses the three authors and order from the supplied manuscript. It deliberately omits an invented DOI, release date, and blanket repository licence. Set the actual release date when publishing; add the Zenodo version DOI after it exists. Check the resulting Zenodo record's metadata and component-specific licence description.

The current upstream MAX31865 Arduino repository calls its licence BSD, but its referenced license.txt was not available in the inspected tree. ADAFRUIT-NOTICES.md preserves the actual upstream notices and does not assign an unverified BSD variant. If the exact incorporated source snapshot contains a full licence, preserve that file too; otherwise obtain clarification from the upstream maintainer before treating the exact BSD terms as resolved.

The supplied firmware identifies its driver as Adafruit-derived but does not include the complete upstream banner near the driver. Copy the full software banner from ADAFRUIT-NOTICES.md into both sketches above the local driver, retaining any other existing notices. MIT applies to original contributions, not as a replacement for that notice.

Dependency instructions are based on the supplied V1.1 files, not a verified current GitHub checkout. Their headers still describe target compilation and bench validation as outstanding. Update those statements only to reflect checks actually completed, and retain the relationship between the experimental firmware and this release in the build record. No board build or physical test was performed while preparing this metadata package.

Enable the renamed repository in Zenodo before publishing the GitHub release. Verify that all files advertised by RELEASE-NOTES.md are included in the tag. Keep future source changes in a new version rather than moving the archived tag.
