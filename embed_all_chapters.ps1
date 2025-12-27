# Embed All Chapters Script (PowerShell)
# Reads all markdown files from docs/chapters and sends them to /embed endpoint

# Configuration
$BackendUrl = "http://localhost:8000"
$ChaptersDir = "frontend\my-book\docs\chapters"
$BookId = "physical-ai-humanoid"

Write-Host "=" * 60
Write-Host "EMBEDDING ALL CHAPTERS"
Write-Host "=" * 60
Write-Host "Backend: $BackendUrl"
Write-Host "Chapters: $ChaptersDir"
Write-Host "Book ID: $BookId"
Write-Host "=" * 60

# Get all markdown files
$ChapterFiles = Get-ChildItem -Path $ChaptersDir -Filter "*.md" | Sort-Object Name

if ($ChapterFiles.Count -eq 0) {
    Write-Host "No chapter files found in $ChaptersDir"
    exit
}

Write-Host "`nFound $($ChapterFiles.Count) chapters`n"

# Counters
$SuccessCount = 0
$FailureCount = 0

# Process each chapter
foreach ($File in $ChapterFiles) {
    $ChapterName = $File.BaseName
    $Content = Get-Content -Path $File.FullName -Raw -Encoding UTF8

    Write-Host "Embedding: $ChapterName..."

    # Create payload
    $Payload = @{
        content = $Content
        chapter = $ChapterName
        book_id = $BookId
    } | ConvertTo-Json

    try {
        # Send POST request
        $Response = Invoke-RestMethod `
            -Uri "$BackendUrl/embed" `
            -Method Post `
            -Body $Payload `
            -ContentType "application/json"

        $ChunksCreated = $Response.chunks_created
        Write-Host "  ✓ Success: $ChunksCreated chunks created" -ForegroundColor Green
        $SuccessCount++
    }
    catch {
        Write-Host "  ✗ Error: $($_.Exception.Message)" -ForegroundColor Red
        $FailureCount++
    }
}

# Summary
Write-Host "`n$("=" * 60)"
Write-Host "SUMMARY"
Write-Host "=" * 60
Write-Host "Total chapters: $($ChapterFiles.Count)"
Write-Host "Successful: $SuccessCount" -ForegroundColor Green
Write-Host "Failed: $FailureCount" -ForegroundColor Red
Write-Host "=" * 60
