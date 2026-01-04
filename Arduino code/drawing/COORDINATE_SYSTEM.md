# Coordinate System and Measurement Reference

## Physical Setup
- **Left Motor**: Position (0, 0) - origin
- **Right Motor**: Position (BASE, 0) = (700, 0)
- **X-axis**: Increases to the right (0 to 700 mm)
- **Y-axis**: Increases downward (0 to 1000 mm)

## Belt Length Components

### Left Side:
- `L1` = Fixed belt length from left motor to left belt gripper (mm)
- `L_ARM` = Fixed arm length from gripper to pen (mm)
- `Z1_i` = Initial total belt length = L1 + L_ARM (mm)
- `Z1` = Current/target total belt length from left motor to pen (mm)

### Right Side:
- `L2` = Fixed belt length from right motor to right belt gripper (mm)
- `L_ARM` = Fixed arm length from gripper to pen (same as left)
- `Z2_i` = Initial total belt length = L2 + L_ARM (mm)
- `Z2` = Current/target total belt length from right motor to pen (mm)

## Inverse Kinematics

For a target position (x, y):
- `Z1 = sqrt(x² + y²)` - Distance from left motor (0,0) to pen
- `Z2 = sqrt((BASE-x)² + y²)` - Distance from right motor (BASE,0) to pen

These are **absolute belt lengths** measured from the motors to the pen.

## Step Position Reference

### Initial State:
- Stepper position = 0 (represents the initial belt length)
- Belt length = Z1_i (for left), Z2_i (for right)
- Position 0 = "home" position

### Movement Calculation:
1. Calculate target belt length: `Z1 = sqrt(x² + y²)`
2. Calculate change: `ΔZ1 = Z1 - Z1_i`
3. Convert to steps: `steps = beltToSteps(ΔZ1) = ΔZ1 / MM_PER_STEP`
4. Calculate absolute target: `targetSteps1 = absoluteSteps1 + steps`
5. Move to absolute position: `moveTo(targetSteps1)`
6. Update: `absoluteSteps1 = targetSteps1`, `Z1_i = Z1`

## Conversion Factors

- `PULLEY_DIAMETER` = 12.7 mm
- `MOTOR_STEPS` = 200 steps/revolution
- `MICROSTEPS` = 1
- `STEPS_PER_REV` = 200 × 1 = 200 steps
- `MM_PER_STEP` = (π × 12.7) / 200 ≈ 0.199 mm/step

## Measurement Summary

| Variable | Unit | Meaning |
|----------|------|---------|
| x, y | mm | Pen position (absolute coordinates) |
| Z1, Z2 | mm | Total belt length from motor to pen (absolute) |
| Z1_i, Z2_i | mm | Current belt length (tracked, updated after each move) |
| absoluteSteps1, absoluteSteps2 | steps | Stepper position (absolute, accumulates) |
| MM_PER_STEP | mm/step | Belt movement per motor step |

**Key Point**: All belt lengths (Z1, Z2, Z1_i, Z2_i) are measured from the motor center to the pen, including both the belt and arm components.

