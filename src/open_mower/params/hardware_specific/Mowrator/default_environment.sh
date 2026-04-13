# Measured Mowrator geometry defaults.
export OM_ANTENNA_OFFSET_X=${OM_ANTENNA_OFFSET_X:-0.72}
export OM_ANTENNA_OFFSET_Y=${OM_ANTENNA_OFFSET_Y:-0.0}

export OM_WHEEL_DISTANCE_M=${OM_WHEEL_DISTANCE_M:-0.58}
export OM_WHEEL_TICKS_PER_M=${OM_WHEEL_TICKS_PER_M:-374.1}
export OM_TOOL_WIDTH=${OM_TOOL_WIDTH:-0.4}

# Keep the verified current UART layout for the first Flipsky bring-up.
export OM_LL_SERIAL_PORT=${OM_LL_SERIAL_PORT:-/dev/ttyAMA0}
export OM_XESC_LEFT_PORT=${OM_XESC_LEFT_PORT:-/dev/ttyAMA5}
export OM_XESC_RIGHT_PORT=${OM_XESC_RIGHT_PORT:-/dev/ttyAMA3}
export OM_XESC_MOWER_PORT=${OM_XESC_MOWER_PORT:-/dev/ttyAMA4}
