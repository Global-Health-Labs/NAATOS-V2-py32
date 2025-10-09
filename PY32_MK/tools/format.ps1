param(
    [string]$ClangFormatBin = "clang-format.exe"
)

Write-Host "Using clang-format: $ClangFormatBin"

$files = @()
$gitOk = $false
try {
    git rev-parse --is-inside-work-tree *> $null
    if ($LASTEXITCODE -eq 0) { $gitOk = $true }
} catch {}

if ($gitOk) {
    $files = git ls-files *.c *.cpp *.h *.hpp
} else {
    $files = Get-ChildItem -Recurse -Include *.c,*.cpp,*.h,*.hpp | ForEach-Object { $_.FullName }
}

if (-not $files -or $files.Count -eq 0) {
    Write-Host "No C/C++ source files found."
    exit 0
}

foreach ($f in $files) {
    & $ClangFormatBin -i $f
}

Write-Host "Formatting complete."