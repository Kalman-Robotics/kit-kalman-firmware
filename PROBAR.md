## ACTUAL

adc:
  battery:
    attenuation: 7.0 # applied voltage divider
    gpio: 8
    voltage: # related to Battery voltage (without reduction)
      full: 1.96
      empty: 1.64

## NUEVO

adc:
  battery:
    attenuation: 7.0 # applied voltage divider
    gpio: 8
    voltage: # related to Battery voltage (without reduction)
      full: 4.1
      empty: 3.5

## ANALIZAR

ros2 topic echo /telemetry --field battery_mv

## ALTERNATIVAS

- [x] probar con otro punto de medición
- [x] revisar código de adc
- [ ] promedio de múltiples lecturas. problema: que muchas sean 0
