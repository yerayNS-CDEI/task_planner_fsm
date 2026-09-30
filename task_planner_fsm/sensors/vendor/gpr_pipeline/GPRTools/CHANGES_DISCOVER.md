# DISCOVER integration note

GPRTools is shared by both GPR interpretation branches. The processing algorithms
are unchanged. Four package/import files were adjusted only to make optional
format dependencies lazy/defensive:

- `loaders/__init__.py`
- `info/__init__.py`
- `info/info_dispatcher.py`
- `plotters/__init__.py`

Reason: importing the SEGY-only OLIWALL path previously imported DZT helpers too,
which forced the unrelated optional `readgssi` dependency even when no DZT file
was used. The DZT/GPRMax modules remain in the package and are still available
when their dependencies are installed.
