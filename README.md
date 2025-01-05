# PID Controller Collection (Speed-Type, Integer(non-float)-Mode) Written in C

## About
- Speed-Type PID Controller.
- Use only integers. floating point is not used.
- Support Direct/Reverse Acting Mode.
- Support PID/PI-D Mode.
- Written in C.

## Target environment
- Microcontroller without FPU.

## Usage
[usage.c](usage.c)

```
$ cc PID.c usage.c && ./a.out
```

## Tuning (References)
- [Ziegler–Nichols method](https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method)
- [CHR method](https://ja.wikipedia.org/wiki/PID%E5%88%B6%E5%BE%A1#CHR%E6%B3%95)
- Step response method

## License
[LICENSE.txt](LICENSE.txt)

