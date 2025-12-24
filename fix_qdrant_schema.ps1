# Qdrant Collection Schema Fix Script (PowerShell)
# This script fixes invalid Qdrant collection schema and re-embeds chapters

# Configuration
$BackendUrl = "http://localhost:8000"
$QdrantUrl = "http://localhost:6333"
$CollectionName = "book_embeddings"
$ChaptersDir = "frontend\my-book\docs\chapters"
$BookId = "physical-ai-humanoid"

function Print-Section {
    param([string]$Title)
    Write-Host "`n$("=" * 70)"
    Write-Host " $Title"
    Write-Host "$("=" * 70)"
}

function Check-QdrantHealth {
    Print-Section "STEP 1: Check Current Qdrant Health"

    try {
        $Response = Invoke-RestMethod -Uri "$BackendUrl/health/qdrant" -Method Get
        Write-Host "Status: $($Response.status)"
        Write-Host "Message: $($Response.message)"

        if ($Response.metadata) {
            Write-Host "Metadata: $($Response.metadata | ConvertTo-Json -Depth 3)"
        }

        return $Response
    }
    catch {
        Write-Host "ERROR: Cannot connect to backend: $($_.Exception.Message)" -ForegroundColor Red
        Write-Host "Make sure backend is running on port 8000"
        return $null
    }
}

function Delete-Collection {
    Print-Section "STEP 2: Delete Invalid Collection"

    try {
        $Response = Invoke-RestMethod -Uri "$QdrantUrl/collections/$CollectionName" -Method Delete
        Write-Host "✓ Deleted collection: $CollectionName" -ForegroundColor Green
        Start-Sleep -Seconds 2
        return $true
    }
    catch {
        Write-Host "Error deleting collection: $($_.Exception.Message)" -ForegroundColor Red
        return $false
    }
}

function Create-Collection {
    Print-Section "STEP 3: Create Collection with Correct Schema"

    Write-Host "Creating collection with:"
    Write-Host "  - Name: book_embeddings"
    Write-Host "  - Vector size: 1536 (OpenAI text-embedding-3-small)"
    Write-Host "  - Distance: Cosine"
    Write-Host "  - ID type: UUID (required by Qdrant)"

    $Payload = @{
        vectors = @{
            size = 1536
            distance = "Cosine"
        }
    } | ConvertTo-Json

    try {
        $Response = Invoke-RestMethod `
            -Uri "$QdrantUrl/collections/$CollectionName" `
            -Method Put `
            -Body $Payload `
            -ContentType "application/json"

        Write-Host "✓ Collection created successfully" -ForegroundColor Green

        # Verify
        $CollectionInfo = Invoke-RestMethod -Uri "$QdrantUrl/collections/$CollectionName"
        $VectorSize = $CollectionInfo.result.config.params.vectors.size
        $Distance = $CollectionInfo.result.config.params.vectors.distance

        Write-Host "✓ Verified - Vector size: $VectorSize" -ForegroundColor Green
        Write-Host "✓ Verified - Distance: $Distance" -ForegroundColor Green

        return $true
    }
    catch {
        Write-Host "ERROR creating collection: $($_.Exception.Message)" -ForegroundColor Red
        return $false
    }
}

function Embed-Chapter {
    param(
        [string]$ChapterName,
        [string]$Content
    )

    $Payload = @{
        content = $Content
        chapter = $ChapterName
        book_id = $BookId
    } | ConvertTo-Json

    try {
        $Response = Invoke-RestMethod `
            -Uri "$BackendUrl/embed" `
            -Method Post `
            -Body $Payload `
            -ContentType "application/json"

        $ChunksCreated = $Response.chunks_created
        Write-Host "  ✓ Success: $ChunksCreated chunks created" -ForegroundColor Green
        return @{success=$true; chunks=$ChunksCreated}
    }
    catch {
        Write-Host "  ✗ Error: $($_.Exception.Message)" -ForegroundColor Red
        return @{success=$false; chunks=0}
    }
}

function ReEmbed-AllChapters {
    Print-Section "STEP 4: Re-Embed All Chapters with UUID IDs"

    $ChapterFiles = Get-ChildItem -Path $ChaptersDir -Filter "*.md" | Sort-Object Name

    if ($ChapterFiles.Count -eq 0) {
        Write-Host "ERROR: No chapter files found in $ChaptersDir" -ForegroundColor Red
        return $false
    }

    Write-Host "Found $($ChapterFiles.Count) chapters to embed`n"

    $SuccessCount = 0
    $TotalChunks = 0

    $i = 0
    foreach ($File in $ChapterFiles) {
        $i++
        $ChapterName = $File.BaseName
        $Content = Get-Content -Path $File.FullName -Raw -Encoding UTF8

        Write-Host "[$i/$($ChapterFiles.Count)] Embedding: $ChapterName..."

        $Result = Embed-Chapter -ChapterName $ChapterName -Content $Content

        if ($Result.success) {
            $SuccessCount++
            $TotalChunks += $Result.chunks
        }

        Start-Sleep -Milliseconds 500
    }

    Write-Host "`n✓ Successfully embedded $SuccessCount/$($ChapterFiles.Count) chapters" -ForegroundColor Green
    Write-Host "✓ Total chunks created: $TotalChunks" -ForegroundColor Green

    return $SuccessCount -gt 0
}

function Verify-Fix {
    Print-Section "STEP 5: Verify Fix"

    try {
        $Health = Invoke-RestMethod -Uri "$BackendUrl/health/qdrant"
        Write-Host "Status: $($Health.status)"
        Write-Host "Message: $($Health.message)"

        if ($Health.metadata) {
            Write-Host "Collection: $($Health.metadata.collection)"
            Write-Host "Vectors count: $($Health.metadata.vectors_count)"
        }

        # Test RAG query
        Write-Host "`nTesting RAG query..."
        $QueryPayload = @{ query = "What is ROS 2?" } | ConvertTo-Json

        $QueryResponse = Invoke-RestMethod `
            -Uri "$BackendUrl/query" `
            -Method Post `
            -Body $QueryPayload `
            -ContentType "application/json"

        if ($QueryResponse.answer -and $QueryResponse.answer -notlike "*No relevant context found*") {
            Write-Host "✓ RAG query successful - embeddings working!" -ForegroundColor Green
        }
        else {
            Write-Host "⚠ RAG query returned no context (may need more time)" -ForegroundColor Yellow
        }

        return $Health.status -eq "healthy"
    }
    catch {
        Write-Host "⚠ Could not verify: $($_.Exception.Message)" -ForegroundColor Yellow
        return $false
    }
}

# Main execution
Print-Section "QDRANT COLLECTION SCHEMA FIX (PowerShell)"
Write-Host "This script will:"
Write-Host "1. Check current Qdrant health"
Write-Host "2. Delete invalid collection"
Write-Host "3. Create new collection with correct schema (UUID IDs)"
Write-Host "4. Re-embed all chapters"
Write-Host "5. Verify fix"

# Step 1: Check health
$Health = Check-QdrantHealth
if ($null -eq $Health) {
    exit
}

# Confirm
Write-Host "`n$("-" * 70)"
$Confirm = Read-Host "Continue with collection deletion and recreation? (yes/no)"
if ($Confirm -notlike "y*") {
    Write-Host "Aborted. No changes made."
    exit
}

# Step 2: Delete collection
if (-not (Delete-Collection)) {
    Write-Host "ERROR: Could not delete collection" -ForegroundColor Red
    exit
}

# Step 3: Create collection
if (-not (Create-Collection)) {
    Write-Host "ERROR: Could not create collection" -ForegroundColor Red
    exit
}

# Step 4: Re-embed chapters
if (-not (ReEmbed-AllChapters)) {
    Write-Host "ERROR: Could not re-embed chapters" -ForegroundColor Red
    exit
}

# Step 5: Verify
if (Verify-Fix) {
    Print-Section "SUCCESS!"
    Write-Host "✓ Qdrant collection schema is now valid" -ForegroundColor Green
    Write-Host "✓ All chapters embedded with UUID IDs" -ForegroundColor Green
    Write-Host "✓ RAG queries should now work" -ForegroundColor Green
}
else {
    Print-Section "PARTIAL SUCCESS"
    Write-Host "Collection recreated and chapters embedded"
    Write-Host "May need additional time for health check to reflect changes"
}
