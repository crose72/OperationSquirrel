# setup_env.ps1
$jsonContent = Get-Content -Path "env_config.json" -Raw
$config = $jsonContent | ConvertFrom-Json

# Set Windows environment variables
foreach ($property in $config.windows.PSObject.Properties) {
    $name = $property.Name
    $value = $property.Value
    [System.Environment]::SetEnvironmentVariable($name, $value, [System.EnvironmentVariableTarget]::Process)
    Write-Host "Set $name = $value"
}

Write-Host "Windows environment variables have been set successfully"
