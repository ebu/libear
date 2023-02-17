# Changelog

## unreleased changes

### Changed

- `Layout::screen` defaults to `getDefaultScreen()` to match the EAR. Call `layout.screen(boost::none)` to get the old behaviour.

- added xsimd submodule and updated eigen to 3.4.0; this required changing the eigen remote, so you may need to run `git submodule sync` as well as the usual `git submodule update --init --recursive`

## 0.9.0

Initial release.
