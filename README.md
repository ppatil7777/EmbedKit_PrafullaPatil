# Embed Kit

Author for submission: Prafulla Patil

`uart_parser.c` implements a byte-at-a-time UART frame parser with XOR checksum verification, configurable inter-byte timeout, and a deterministic stream demonstration.

## Build and run

```sh
gcc -Wall -std=c99 uart_parser.c -o uart_parser
./uart_parser
```

On Windows with MinGW GCC, run `./uart_parser.exe` from PowerShell after compiling. The program uses only the standard C library and returns a nonzero exit status if any scenario produces an unexpected count of frames or errors.

## Demonstration

| Scenario | Expected result |
| --- | --- |
| Clean frame | Valid at 30 ms, command `01`, payload `10 20 30` |
| Timeout and recovery | 185 ms gap detected at 200 ms, followed by a valid frame at 220 ms |
| Consecutive frames | Valid frames at 20 and 50 ms, including `AA` as a payload byte |
| Timeout disabled | Zero timeouts and a checksum error at 210 ms |

## How the parser works

Each call handles one byte. The parser first looks for the start marker, then reads the command, length, payload, and checksum in order. The length tells it how many payload bytes to read. The example program passes timestamps from an array, so all four demonstrations run without UART hardware or delays.

## Protocol details

- The checksum XORs CMD, LEN, and the payload. For the clean frame, `01 ^ 03 ^ 10 ^ 20 ^ 30 = 02`, so the demonstration uses `02` where the assignment prints `22`.
- The recovery checksum is `05 ^ 01 ^ 7F = 7B`. The consecutive frame checksums are `03 ^ 01 ^ 55 = 57` and `04 ^ 02 ^ AA ^ BB = 17`.
- The timeout comparison uses a strict greater-than test. On a timeout, the feed function resets and returns `-2`. The stream helper then re-feeds the same byte at the same timestamp, accounting for the repeated 200 ms entry in the assignment.
- Lengths above 16 reset the parser to the start-of-frame search. A zero length goes directly to checksum verification.
