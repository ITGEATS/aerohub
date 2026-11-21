param(
    [Parameter(Mandatory)][string]$SettingsFile,
    [Parameter(Mandatory)][string]$Network,
    [Parameter(Mandatory)][string]$ContainerName,
    [string]$AirSimHost = '192.168.65.254',
    [string]$LoggerTopicsFile
)

$ErrorActionPreference = 'Stop'
$settings = (Resolve-Path -LiteralPath $SettingsFile).Path
$dockerArgs = @(
    'run', '--detach', '--pull', 'never', '--name', $ContainerName, '--network', $Network,
    '--env', 'ROS_DOMAIN_ID=161', '--env', 'ROS_LOCALHOST_ONLY=0',
    '--env', "AIRSIM_HOST=$AirSimHost", '--env', 'AIRSIM_RPC_PORT=43518',
    '--env', 'VEHICLE_NAME=PX4', '--env', 'VEHICLE_NAMESPACE=drone0',
    '--env', 'PX4_INSTANCE=96', '--env', 'PX4_SYSTEM_ID=97', '--env', 'XRCE_AGENT_PORT=9018',
    '--env', 'FLIGHT_BOARD_SENSORS=0', '--env', 'FLIGHT_BOARD_CAMERAS=0',
    # Match the AirSim GPS model's 200 ms latency using PX4's startup override.
    '--env', 'PX4_PARAM_SENS_GPS0_DELAY=200',
    '--mount', "type=bind,source=$settings,target=/config/settings.json,readonly"
)

if ($LoggerTopicsFile) {
    $topics = (Resolve-Path -LiteralPath $LoggerTopicsFile).Path
    $dockerArgs += @('--mount', "type=bind,source=$topics,target=/var/lib/flight_board/fs/etc/logging/logger_topics.txt,readonly")
}

docker @dockerArgs ghcr.io/popc0rn7/aerohub/flight-board:v0.1.0 vehicle
if ($LASTEXITCODE -ne 0) {
    throw "Could not start PX4 container $ContainerName"
}
