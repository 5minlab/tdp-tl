
$UE_PROJ_DIR = $env:UE_PROJ_DIR ?? "D:\unreal\tdp1"

function build {
  cargo build --release
}

function install {
  cp ./target/release/tdp_tl.dll "${UE_PROJ_DIR}\Plugins\Tdp\Binaries\Win64\tdp_tl.dll"
  cp Bindings.h "${UE_PROJ_DIR}\Plugins\Tdp\Source\Tdp\Public\Bindings.h"
}

while ($args.Count -gt 0) {
    & $args[0]
    $args = $args[1..$args.Count]
}
