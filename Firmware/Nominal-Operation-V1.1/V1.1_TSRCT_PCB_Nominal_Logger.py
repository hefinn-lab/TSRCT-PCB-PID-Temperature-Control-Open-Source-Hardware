"""
TSRCT-PCB Nominal Dual-Channel Serial Logger V1.1
Companion to V1.1_TSRCT_PCB_Nominal_Dual_Channel.ino; based on the supplied V2 logger.
Python 3.9+; Arduino Nano Every at 115200 baud. Use this matched firmware pair.

Quick start (close Arduino Serial Monitor first):
    python V1.1_TSRCT_PCB_Nominal_Logger.py --port COM4
    python V1.1_TSRCT_PCB_Nominal_Logger.py --port /dev/ttyACM0 --output-dir logs
Rebuild a workbook after an interrupted session:
    python V1.1_TSRCT_PCB_Nominal_Logger.py --rebuild path/to/capture.csv

V1.1 changes
------------
- CRC-16/CCITT-FALSE, exact 55-field schema, finite/range/enum validation.
- Bad transport frames never enter CSV, Raw Data or plotted measurements.
- Valid sensor-fault rows retain NaN (not a fabricated or repeated temperature).
- A telemetry sequence counts gaps, rejects duplicates and marks board resets.
- Live .session.jsonl preserves metadata, events and rejected-frame diagnostics.
- Rebuild reads the CSV and its optional journal; partial trailing records are skipped.
- Plot gaps remain gaps, channel setpoints appear only during active control,
  and GPIO traces are labelled as scheduled duty, not measured heater power.
- Captures remain passive; port opening can reset some board/USB combinations.
- CSV/journal flush every 5 s, fsync every 30 s and clean shutdown. A hard power
  loss can still lose buffered data. Excel is built on shutdown, not continuously.

Purpose
-------
- Passively records the firmware's continuous 1 Hz D-prefixed telemetry.
- Writes every accepted row immediately to a crash-resistant CSV mirror.
- Continues until Ctrl+C or a serial/contract error; an Arduino ALARM does NOT
  stop logging.
- On termination, creates a formatted XLSX workbook containing:
    * Summary
    * Configuration
    * Events
    * Raw Data (all accepted firmware telemetry + PC timestamp/elapsed time)
    * Plots (temperature vs time and duty vs time for both channels)
- The XLSX plot sheet uses down-sampled plotting data when required, but the Raw
  Data sheet(s) retain every accepted row.

Safety architecture
-------------------
This program is deliberately passive. It does not send commands to the Arduino
and it is not part of the heater safety chain. Closing this logger must not alter
controller operation.

Required packages
-----------------
    pip install pyserial openpyxl

Author: TSRCT / UTAS nominal-operation toolchain
"""

from __future__ import annotations

import argparse
import binascii
import csv
import json
import math
import os
import re
import signal
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

try:
    import serial
except ImportError:
    serial = None

from openpyxl import Workbook
from openpyxl.chart import ScatterChart, Reference, Series
from openpyxl.styles import Alignment, Font, PatternFill
from openpyxl.utils import get_column_letter


# =============================================================================
# USER CONFIGURATION
# =============================================================================
SERIAL_PORT = "COM4"
BAUD_RATE = 115200
SERIAL_TIMEOUT_S = 2.0

# Files are written to the directory from which this script is launched.
OUTPUT_DIRECTORY = Path.cwd()
FILE_PREFIX = "V1.1_TSRCT_PCB_NOMINAL"

# CSV is the live/crash-safe acquisition file. XLSX is generated at shutdown.
CSV_FLUSH_INTERVAL_S = 5.0
CSV_FSYNC_INTERVAL_S = 30.0

# Maximum number of points used in each Excel chart. Raw data are never
# down-sampled; only the hidden Plot Data sheet is reduced for fast rendering.
PLOT_MAX_POINTS = 20_000

# If the firmware prints a # FIELDS contract that differs from this logger's
# known contract, stop rather than silently assigning data to the wrong columns.
STRICT_FIELD_CONTRACT = True

# Malformed-row diagnostics. Rejected D rows are never written into the normal
# telemetry table; they are recorded separately with the exact parser failure.
PRINT_REJECTED_RAW_ROWS = True
MAX_REJECTED_ROWS_IN_XLSX = 10_000

# Excel worksheet row limit. Additional raw-data sheets are created if required.
EXCEL_MAX_ROWS = 1_048_576

LOGGER_VERSION = "TSRCT-PCB Nominal Logger V1.1"
MAX_FRAME_BYTES = 4096
EXPECTED_SCHEMA_VERSION = "1.1"


# =============================================================================
# NOMINAL FIRMWARE SERIAL CONTRACT
# Must match V1.1_TSRCT_PCB_Nominal_Dual_Channel.ino exactly.
# =============================================================================
FIRMWARE_FIELDS = [
    "uptime_s",
    "active_elapsed_s",
    "sys_state",
    "alarm",
    "fault_flags",
    "ch1_mode",
    "ch2_mode",
    "ch1_C",
    "ch2_C",
    "ch1_setpoint_C",
    "ch2_setpoint_C",
    "ch1_target_C",
    "ch2_target_C",
    "ch1_duty_pct",
    "ch2_duty_pct",
    "ch1_scheduled_gpio_duty_pct",
    "ch2_scheduled_gpio_duty_pct",
    "ch1_p_pct",
    "ch1_i_pct",
    "ch1_d_pct",
    "ch2_p_pct",
    "ch2_i_pct",
    "ch2_d_pct",
    "ch1_unclamped_pct",
    "ch2_unclamped_pct",
    "ch1_saturation",
    "ch2_saturation",
    "ch1_i_clamp_state",
    "ch2_i_clamp_state",
    "ch1_i_clamp_active",
    "ch2_i_clamp_active",
    "ch1_filtered_dydt_C_per_s",
    "ch2_filtered_dydt_C_per_s",
    "control_dt_s",
    "windows_passed",
    "control_gap_active",
    "missed_window_count",
    "scheduled_window_us",
    "ch1_ssr_high_us",
    "ch2_ssr_high_us",
    "rtd_state",
    "ch1_rtd_age_ms",
    "ch2_rtd_age_ms",
    "ch1_rtd_sample_count",
    "ch2_rtd_sample_count",
    "ch1_rtd_failure_count",
    "ch2_rtd_failure_count",
    "ch1_rtd_fault_code",
    "ch2_rtd_fault_code",
    "loop_max_us",
    "telemetry_seq",
    "control_window_sequence",
    "completed_window_sequence",
    "ch1_i_hold_active",
    "ch2_i_hold_active",
]

INTEGER_FIELDS = {
    "sys_state",
    "alarm",
    "fault_flags",
    "ch1_mode",
    "ch2_mode",
    "ch1_saturation",
    "ch2_saturation",
    "ch1_i_clamp_state",
    "ch2_i_clamp_state",
    "ch1_i_clamp_active",
    "ch2_i_clamp_active",
    "windows_passed",
    "control_gap_active",
    "missed_window_count",
    "scheduled_window_us",
    "ch1_ssr_high_us",
    "ch2_ssr_high_us",
    "rtd_state",
    "ch1_rtd_age_ms",
    "ch2_rtd_age_ms",
    "ch1_rtd_sample_count",
    "ch2_rtd_sample_count",
    "ch1_rtd_failure_count",
    "ch2_rtd_failure_count",
    "ch1_rtd_fault_code",
    "ch2_rtd_fault_code",
    "loop_max_us",
    "telemetry_seq",
    "control_window_sequence",
    "completed_window_sequence",
    "ch1_i_hold_active",
    "ch2_i_hold_active",
}

CSV_FIELDS = ["Timestamp", "Logger_elapsed_s", "Logger_elapsed_h"] + FIRMWARE_FIELDS
EXPECTED_PARTS = 1 + len(FIRMWARE_FIELDS)  # D prefix + firmware values

RE_EVENT = re.compile(r"^#\s*EVENT:\s*(?P<body>.+)$")
RE_CONFIG = re.compile(r"^#\s*CONFIG:\s*(?P<body>.+)$")
RE_FIELDS = re.compile(r"^#\s*FIELDS:\s*(?P<body>.+)$")
RE_FIRMWARE = re.compile(r"^#\s*(?:FIRMWARE:\s*)?(?P<name>TSRCT-PCB Nominal Dual-Channel Controller V1\.1)\s*$")

SYSTEM_STATE_NAMES = {0: "IDLE", 1: "ACTIVE", 2: "ALARM"}
CHANNEL_MODE_NAMES = {0: "OFF", 1: "SENSE_ONLY", 2: "CONTROL"}


# =============================================================================
# PARSING HELPERS
# =============================================================================
def validate_row(row: Dict[str, object]) -> Optional[str]:
    """Validate schema values; only the two sensor temperatures may be NaN."""
    for field in FIRMWARE_FIELDS:
        value = row[field]
        if field in INTEGER_FIELDS:
            if isinstance(value, bool) or not isinstance(value, int):
                return f"integer_type: {field}"
            if field.endswith(("_saturation", "_i_clamp_state")):
                valid = value in (-1, 0, 1)
            elif field in ("sys_state", "rtd_state", "ch1_mode", "ch2_mode"):
                valid = value in (0, 1, 2)
            elif field == "alarm" or field.endswith(("_active", "_i_hold_active")):
                valid = value in (0, 1)
            elif field == "fault_flags":
                valid = 0 <= value <= 0x01FF
            elif field.endswith(("_rtd_failure_count", "_rtd_fault_code")) or field == "windows_passed":
                valid = 0 <= value <= 255
            elif field == "scheduled_window_us":
                valid = value == 1_000_000
            elif field.endswith("_ssr_high_us"):
                valid = 0 <= value <= 1_000_000
            else:
                valid = 0 <= value <= 0xFFFFFFFF
            if not valid:
                return f"integer_range: {field}={value}"
        else:
            if field in ("ch1_C", "ch2_C") and math.isnan(float(value)):
                continue
            if not math.isfinite(float(value)):
                return f"nonfinite_value: {field}"
            if field.endswith("_C") and not -20 <= float(value) <= 200:
                return f"temperature_range: {field}={value}"
            if field.endswith(("_duty_pct", "_i_pct")) and not 0 <= float(value) <= 100:
                return f"output_range: {field}={value}"
            if field in ("uptime_s", "control_dt_s") and float(value) < 0:
                return f"negative_time: {field}"
            if field == "active_elapsed_s" and float(value) < 0 and float(value) != -1:
                return "invalid_active_elapsed_s"
    if int(row["alarm"]) != int(int(row["sys_state"]) == 2):
        return "inconsistent_alarm_state"
    for ch in (1, 2):
        if int(row[f"ch{ch}_mode"]) == 0 and not math.isnan(float(row[f"ch{ch}_C"])):
            return f"temperature_on_disabled_channel: ch{ch}"
        if int(row[f"ch{ch}_mode"]) != 2 or int(row["sys_state"]) != 1:
            if float(row[f"ch{ch}_duty_pct"]) != 0 or float(row[f"ch{ch}_scheduled_gpio_duty_pct"]) != 0:
                return f"duty_on_inactive_channel: ch{ch}"
    return None


def parse_data_line_diagnostic(
    line: str,
) -> Tuple[Optional[Dict[str, object]], Optional[str], int]:
    """Verify ASCII frame CRC, field order/count, numeric syntax and value bounds.

    Format: D,<55 values>*HHHH. CRC covers exactly the bytes preceding '*'.
    CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no xor-out.
    CRC detects transport corruption; it does not authenticate a sender.
    """
    line = line.rstrip("\r\n")
    parts_count = line.count(",") + 1
    if len(line) > MAX_FRAME_BYTES:
        return None, "oversized_frame", parts_count
    if not line.startswith("D,"):
        return None, "not_D_prefix", parts_count
    try:
        encoded = line.encode("ascii")
    except UnicodeEncodeError:
        return None, "decode_replacement_character_present", parts_count
    if any(b < 32 or b > 126 for b in encoded):
        return None, "decode_control_character_present", parts_count
    payload, separator, checksum = line.rpartition("*")
    if not separator or not re.fullmatch(r"[0-9A-Fa-f]{4}", checksum):
        return None, "crc_framing_missing_or_invalid", parts_count
    if binascii.crc_hqx(payload.encode("ascii"), 0xFFFF) != int(checksum, 16):
        return None, "crc_mismatch", parts_count
    parts = payload.split(",")
    if len(parts) != EXPECTED_PARTS:
        return None, f"field_count_mismatch: got={len(parts)}, expected={EXPECTED_PARTS}", len(parts)
    row: Dict[str, object] = {}
    for field, value in zip(FIRMWARE_FIELDS, parts[1:]):
        try:
            if field in INTEGER_FIELDS:
                if not re.fullmatch(r"-?[0-9]+", value):
                    raise ValueError("expected decimal integer")
                row[field] = int(value)
            else:
                if value != "nan" and not re.fullmatch(r"-?(?:[0-9]+(?:\.[0-9]+)?)(?:[eE][+-]?[0-9]+)?", value):
                    raise ValueError("expected finite decimal or sensor nan")
                row[field] = float(value)
        except (ValueError, OverflowError):
            return None, f"numeric_parse_failure: {field}={value!r}", len(parts)
    error = validate_row(row)
    return (None if error else row), error, len(parts)


def journal_path_for(csv_path: Path) -> Path:
    return csv_path.with_suffix(".session.jsonl")


def iter_csv_rows(csv_path: Path) -> Iterable[Dict[str, object]]:
    """Rebuild accepts complete validated CSV rows; no interpolation/imputation.

    The live parser already verified CRC before writing CSV. A CSV has no frame
    checksum, so this is structural/range checking, not post-edit CRC verification.
    """
    with csv_path.open("r", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames != CSV_FIELDS:
            raise RuntimeError("CSV header does not match the V1.1 logger contract.")
        for record in reader:
            if None in record or any(record.get(name) is None for name in CSV_FIELDS):
                continue
            try:
                row = {field: int(record[field]) if field in INTEGER_FIELDS
                       else float(record[field]) for field in FIRMWARE_FIELDS}
                if validate_row(row):
                    continue
                elapsed = float(record["Logger_elapsed_s"])
                hours = float(record["Logger_elapsed_h"])
                if not math.isfinite(elapsed) or not math.isfinite(hours) or elapsed < 0 or hours < 0:
                    continue
                datetime.fromisoformat(record["Timestamp"])
                row.update(Timestamp=record["Timestamp"], Logger_elapsed_s=elapsed,
                           Logger_elapsed_h=hours)
            except (ValueError, TypeError, OverflowError):
                continue
            yield row


def parse_data_line(line: str) -> Optional[Dict[str, object]]:
    """Compatibility wrapper returning only the parsed row."""
    row, _reason, _parts = parse_data_line_diagnostic(line)
    return row


def parse_config_pairs(body: str) -> List[Tuple[str, str]]:
    """Convert a # CONFIG comma-separated key=value list into clean rows."""
    pairs: List[Tuple[str, str]] = []
    for part in body.split(","):
        part = part.strip()
        if "=" not in part:
            continue
        key, value = part.split("=", 1)
        pairs.append((key.strip(), value.strip()))
    return pairs


def fmt_number(value: object, width: int = 7, precision: int = 3) -> str:
    """Compact terminal formatter that tolerates NaN."""
    try:
        x = float(value)
    except (TypeError, ValueError):
        return "--"
    if not math.isfinite(x):
        return "nan"
    return f"{x:{width}.{precision}f}"


# =============================================================================
# FILE PATHS
# =============================================================================
def resolve_output_paths() -> Tuple[Path, Path]:
    OUTPUT_DIRECTORY.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    base = OUTPUT_DIRECTORY / f"{FILE_PREFIX}_{stamp}"

    csv_path = Path(str(base) + ".csv")
    xlsx_path = Path(str(base) + ".xlsx")

    counter = 1
    while csv_path.exists() or xlsx_path.exists():
        base_n = OUTPUT_DIRECTORY / f"{FILE_PREFIX}_{stamp}_{counter}"
        csv_path = Path(str(base_n) + ".csv")
        xlsx_path = Path(str(base_n) + ".xlsx")
        counter += 1

    return csv_path, xlsx_path


# =============================================================================
# XLSX FORMATTING HELPERS
# =============================================================================
HEADER_FILL = PatternFill("solid", fgColor="1F4E78")
HEADER_FONT = Font(color="FFFFFF", bold=True)
SUBHEADER_FILL = PatternFill("solid", fgColor="D9EAF7")


def style_header(ws, row: int = 1) -> None:
    for cell in ws[row]:
        if cell.value is None:
            continue
        cell.fill = HEADER_FILL
        cell.font = HEADER_FONT
        cell.alignment = Alignment(horizontal="center", vertical="center")


def set_reasonable_widths(ws, widths: Dict[str, float]) -> None:
    for column, width in widths.items():
        ws.column_dimensions[column].width = width


def add_key_value_sheet(wb: Workbook, title: str, rows: Sequence[Sequence[object]]) -> None:
    ws = wb.create_sheet(title)
    for row in rows:
        ws.append(list(row))
    if rows:
        style_header(ws, 1)
        ws.freeze_panes = "A2"
        ws.auto_filter.ref = ws.dimensions
    set_reasonable_widths(ws, {"A": 26, "B": 28, "C": 65})


def excel_value(header: str, value: str) -> object:
    """Convert CSV text back to typed Excel cells while preserving explicit nan."""
    if header == "Timestamp":
        return value
    if value == "":
        return None
    if value.lower() == "nan":
        return "nan"
    if header in INTEGER_FIELDS:
        try:
            return int(value)
        except ValueError:
            return value
    try:
        return float(value)
    except ValueError:
        return value


def plot_numeric(value: str) -> Optional[float]:
    try:
        x = float(value)
    except (TypeError, ValueError):
        return None
    return x if math.isfinite(x) else None


def create_raw_sheet(wb: Workbook, index: int) -> object:
    title = "Raw Data" if index == 1 else f"Raw Data {index}"
    ws = wb.create_sheet(title)
    ws.append(CSV_FIELDS)
    style_header(ws)
    ws.freeze_panes = "A2"
    ws.auto_filter.ref = f"A1:{get_column_letter(len(CSV_FIELDS))}1"

    # Widths are deliberately bounded because this sheet has >50 columns.
    for col_idx, heading in enumerate(CSV_FIELDS, start=1):
        if heading == "Timestamp":
            width = 26
        elif heading in ("Logger_elapsed_s", "Logger_elapsed_h"):
            width = 18
        else:
            width = min(max(len(heading) + 2, 12), 26)
        ws.column_dimensions[get_column_letter(col_idx)].width = width
    return ws


def add_plot_series(chart: ScatterChart,
                    ws,
                    x_col: int,
                    y_col: int,
                    max_row: int,
                    title: str) -> None:
    if max_row < 2:
        return
    xvalues = Reference(ws, min_col=x_col, min_row=2, max_row=max_row)
    yvalues = Reference(ws, min_col=y_col, min_row=2, max_row=max_row)
    series = Series(yvalues, xvalues, title=title)
    series.marker.symbol = "none"
    chart.series.append(series)


def create_plots_sheet(wb: Workbook, plot_ws, plot_rows: int, stride: int) -> None:
    ws = wb.create_sheet("Plots")
    ws["A1"] = "TSRCT-PCB Nominal Operation — Time Series"
    ws["A1"].font = Font(bold=True, size=14)
    ws["A2"] = (
        f"Charts use every {stride} accepted row(s); raw workbook data remain complete."
    )

    if plot_rows < 1:
        ws["A4"] = "No accepted telemetry rows were available for plotting."
        return

    # Plot Data columns:
    # A time_h, B ch1_C, C ch1_setpoint, D ch2_C, E ch2_setpoint,
    # F ch1_duty, G ch1_actual, H ch2_duty, I ch2_actual
    max_row = plot_rows + 1

    temp_chart = ScatterChart()
    temp_chart.title = "Temperature vs Time — Both Channels"
    temp_chart.style = 13
    temp_chart.scatterStyle = "line"
    temp_chart.height = 10
    temp_chart.width = 24
    temp_chart.x_axis.title = "Logger elapsed time (h)"
    temp_chart.y_axis.title = "Temperature (°C)"
    temp_chart.legend.position = "b"

    add_plot_series(temp_chart, plot_ws, 1, 2, max_row, "CH1 Temperature")
    add_plot_series(temp_chart, plot_ws, 1, 3, max_row, "CH1 Setpoint")
    add_plot_series(temp_chart, plot_ws, 1, 4, max_row, "CH2 Temperature")
    add_plot_series(temp_chart, plot_ws, 1, 5, max_row, "CH2 Setpoint")

    duty_chart = ScatterChart()
    duty_chart.title = "SSR Duty vs Time — Both Channels"
    duty_chart.style = 13
    duty_chart.scatterStyle = "line"
    duty_chart.height = 10
    duty_chart.width = 24
    duty_chart.x_axis.title = "Logger elapsed time (h)"
    duty_chart.y_axis.title = "Duty (%)"
    duty_chart.y_axis.scaling.min = 0
    duty_chart.y_axis.scaling.max = 100
    duty_chart.legend.position = "b"

    add_plot_series(duty_chart, plot_ws, 1, 6, max_row, "CH1 Commanded Duty")
    add_plot_series(duty_chart, plot_ws, 1, 7, max_row, "CH1 Scheduled GPIO Duty")
    add_plot_series(duty_chart, plot_ws, 1, 8, max_row, "CH2 Commanded Duty")
    add_plot_series(duty_chart, plot_ws, 1, 9, max_row, "CH2 Scheduled GPIO Duty")

    # Explicit row heights/anchors give each 10 cm chart >2 cm clearance.
    for row in range(1, 62):
        ws.row_dimensions[row].height = 15
    ws.sheet_view.showGridLines = False
    for chart in (temp_chart, duty_chart):
        chart.visible_cells_only = False  # Plot Data is intentionally hidden.
        chart.display_blanks = "gap"
        chart.x_axis.axPos = "b"
        chart.y_axis.axPos = "l"
    ws.add_chart(temp_chart, "A4")
    ws.add_chart(duty_chart, "A30")


def build_workbook(csv_path: Path,
                   xlsx_path: Path,
                   metadata: Dict[str, object],
                   config_rows: Sequence[Dict[str, str]],
                   events: Sequence[Dict[str, str]],
                   rejected_rows: Sequence[Dict[str, object]]) -> None:
    """Build the final XLSX from the crash-safe CSV mirror."""
    wb = Workbook()
    wb.remove(wb.active)

    summary_ws = wb.create_sheet("Summary")
    summary_rows = [
        ("Item", "Value"),
        ("Experiment", "TSRCT-PCB nominal dual-channel operation"),
        ("Logger", LOGGER_VERSION),
        ("Firmware", metadata.get("firmware", "unknown")),
        ("Serial", f"{metadata.get('port', SERIAL_PORT)} @ {BAUD_RATE}"),
        ("Session started", metadata.get("started_at", "")),
        ("Session ended", metadata.get("ended_at", "")),
        ("Termination", metadata.get("termination", "unknown")),
        ("Accepted telemetry rows", metadata.get("accepted_rows", 0)),
        ("Rejected telemetry frames", metadata.get("bad_data_rows", 0)),
        ("Rejected checksum/framing rows", metadata.get("bad_crc_rows", 0)),
        ("Missing telemetry sequences", metadata.get("missing_sequences", 0)),
        ("Board resets observed", metadata.get("board_resets", 0)),
        ("Schema verified", metadata.get("schema_verified", False)),
        ("Journal", str(journal_path_for(csv_path))),
        ("GPIO duty meaning", "Scheduler bookkeeping; no independent pin/power measurement"),
        ("Rejected field-count rows", metadata.get("bad_field_count_rows", 0)),
        ("Rejected numeric-parse rows", metadata.get("bad_numeric_rows", 0)),
        ("Rejected decode-corruption rows", metadata.get("bad_decode_rows", 0)),
        ("Alarm telemetry rows", metadata.get("alarm_rows", 0)),
        ("Control-gap telemetry rows", metadata.get("control_gap_rows", 0)),
        ("CSV mirror", str(csv_path.resolve())),
        ("CSV flush interval (s)", CSV_FLUSH_INTERVAL_S),
        ("CSV fsync interval (s)", CSV_FSYNC_INTERVAL_S),
        ("Plot maximum points", PLOT_MAX_POINTS),
        ("Logging behavior", "Continues through IDLE, ACTIVE and ALARM until Ctrl+C/serial failure"),
        ("Control behavior", "Logger is passive; Arduino retains all control and safety authority"),
    ]
    for row in summary_rows:
        summary_ws.append(row)
    style_header(summary_ws)
    summary_ws.freeze_panes = "A2"
    set_reasonable_widths(summary_ws, {"A": 32, "B": 90})

    config_ws = wb.create_sheet("Configuration")
    config_ws.append(["Timestamp", "Key", "Value"])
    for record in config_rows:
        config_ws.append([record["Timestamp"], record["Key"], record["Value"]])
    style_header(config_ws)
    config_ws.freeze_panes = "A2"
    config_ws.auto_filter.ref = config_ws.dimensions
    set_reasonable_widths(config_ws, {"A": 26, "B": 32, "C": 42})

    events_ws = wb.create_sheet("Events")
    events_ws.append(["Timestamp", "Event"])
    for record in events:
        events_ws.append([record["Timestamp"], record["Event"]])
    style_header(events_ws)
    events_ws.freeze_panes = "A2"
    events_ws.auto_filter.ref = events_ws.dimensions
    set_reasonable_widths(events_ws, {"A": 26, "B": 110})

    rejected_ws = wb.create_sheet("Rejected Rows")
    rejected_ws.append([
        "Timestamp", "Chars", "Parts", "Expected Parts", "Reason", "Raw Line"
    ])
    for record in rejected_rows[:MAX_REJECTED_ROWS_IN_XLSX]:
        rejected_ws.append([
            record.get("Timestamp", ""),
            record.get("Chars", 0),
            record.get("Parts", 0),
            EXPECTED_PARTS,
            record.get("Reason", ""),
            record.get("Raw Line", ""),
        ])
    style_header(rejected_ws)
    rejected_ws.freeze_panes = "A2"
    rejected_ws.auto_filter.ref = rejected_ws.dimensions
    set_reasonable_widths(
        rejected_ws, {"A": 26, "B": 10, "C": 10, "D": 15, "E": 70, "F": 120}
    )

    # Count validated rows, including when rebuilding after an unclean shutdown.
    accepted_rows = sum(1 for _ in iter_csv_rows(csv_path))
    stride = max(1, math.ceil(accepted_rows / PLOT_MAX_POINTS)) if accepted_rows else 1

    plot_ws = wb.create_sheet("Plot Data")
    plot_headers = [
        "Elapsed_h",
        "CH1_C",
        "CH1_Setpoint_C",
        "CH2_C",
        "CH2_Setpoint_C",
        "CH1_Commanded_Duty_pct",
        "CH1_Scheduled_GPIO_Duty_pct",
        "CH2_Commanded_Duty_pct",
        "CH2_Scheduled_GPIO_Duty_pct",
    ]
    plot_ws.append(plot_headers)
    style_header(plot_ws)

    raw_sheet_index = 1
    raw_ws = create_raw_sheet(wb, raw_sheet_index)
    data_rows_on_sheet = 0
    plot_rows = 0

    previous = None
    gap_pending = False
    selected_last = False
    for accepted_index, row in enumerate(iter_csv_rows(csv_path)):
        if data_rows_on_sheet >= EXCEL_MAX_ROWS - 1:
            raw_sheet_index += 1
            raw_ws = create_raw_sheet(wb, raw_sheet_index)
            data_rows_on_sheet = 0
        raw_ws.append([excel_value(name, str(row[name])) for name in CSV_FIELDS])
        data_rows_on_sheet += 1
        if previous is not None:
            delta = (int(row["telemetry_seq"]) - int(previous["telemetry_seq"])) & 0xFFFFFFFF
            gap_pending |= (delta != 1 or float(row["uptime_s"]) <= float(previous["uptime_s"])
                            or float(row["uptime_s"]) - float(previous["uptime_s"]) > 1.5)
        # Missing measurements must not disappear during plot downsampling.
        gap_pending |= any(int(row[f"ch{ch}_mode"]) != 0 and
                           not math.isfinite(float(row[f"ch{ch}_C"])) for ch in (1, 2))
        selected_last = accepted_index % stride == 0 or accepted_index == accepted_rows - 1
        if selected_last:
            x = float(row["Logger_elapsed_h"])
            if gap_pending:
                plot_ws.append([x] + [None] * 8)
                plot_rows += 1
            values = [x]
            for ch in (1, 2):
                values.extend([
                    plot_numeric(str(row[f"ch{ch}_C"])),
                    float(row[f"ch{ch}_setpoint_C"]) if int(row["sys_state"]) == 1
                    and int(row[f"ch{ch}_mode"]) == 2 else None,
                ])
            for ch in (1, 2):
                values.extend([float(row[f"ch{ch}_duty_pct"]),
                               float(row[f"ch{ch}_scheduled_gpio_duty_pct"])])
            plot_ws.append(values)
            plot_rows += 1
            gap_pending = False
        previous = row

    for sheet in wb:
        if sheet.title.startswith("Raw Data"):
            sheet.auto_filter.ref = sheet.dimensions
            for cells in sheet.iter_rows(min_row=2):
                for heading, cell in zip(CSV_FIELDS, cells):
                    if heading in INTEGER_FIELDS:
                        cell.number_format = "0"
                    elif heading.endswith("_C"):
                        cell.number_format = "0.0000"
                    elif isinstance(cell.value, float):
                        cell.number_format = "0.000"
    for sheet in (summary_ws, config_ws, events_ws, rejected_ws):
        for cells in sheet:
            for cell in cells:
                if isinstance(cell.value, str):
                    # Startup separators and received text must stay literal text.
                    cell.data_type = "s"
                    cell.alignment = Alignment(vertical="top", wrap_text=True)
    for cells in summary_ws.iter_rows(min_row=2):
        summary_ws.row_dimensions[cells[0].row].height = 30

    plot_ws.freeze_panes = "A2"
    set_reasonable_widths(
        plot_ws,
        {"A": 15, "B": 14, "C": 18, "D": 14, "E": 18,
         "F": 24, "G": 26, "H": 24, "I": 26},
    )

    create_plots_sheet(wb, plot_ws, plot_rows, stride)
    plot_ws.sheet_state = "hidden"

    # Put the operationally useful sheets first.
    desired_order = ["Summary", "Configuration", "Events", "Rejected Rows", "Plots"]
    desired_order += [
        name for name in wb.sheetnames if name.startswith("Raw Data")
    ]
    desired_order += ["Plot Data"]
    wb._sheets = [wb[name] for name in desired_order]

    # Replace only after a complete ZIP has been written successfully.
    temporary = Path(str(xlsx_path) + ".tmp.xlsx")
    try:
        wb.save(temporary)
        os.replace(temporary, xlsx_path)
    finally:
        if temporary.exists():
            temporary.unlink()


# =============================================================================
# SERIAL LOGGER
# =============================================================================
def new_metadata() -> Dict[str, object]:
    return {
        "firmware": "V1.1-compatible telemetry; announcement not yet received",
        "port": SERIAL_PORT,
        "started_at": datetime.now().isoformat(timespec="seconds"),
        "ended_at": "", "termination": "serial_interruption",
        "accepted_rows": 0, "bad_data_rows": 0, "bad_crc_rows": 0,
        "bad_field_count_rows": 0, "bad_numeric_rows": 0, "bad_decode_rows": 0,
        "alarm_rows": 0, "control_gap_rows": 0, "missing_sequences": 0,
        "board_resets": 0, "schema_verified": False,
    }


def run_logger() -> int:
    if serial is None:
        raise SystemExit("pyserial is not installed. Run: pip install pyserial openpyxl")
    csv_path, xlsx_path = resolve_output_paths()
    journal_path = journal_path_for(csv_path)
    metadata = new_metadata()
    config_rows: List[Dict[str, str]] = []
    events: List[Dict[str, str]] = []
    rejected_rows: List[Dict[str, object]] = []
    last_config: Dict[str, str] = {}
    stop_requested = False
    exit_code = 0
    previous_row = None
    session_start = None
    last_flush = last_fsync = time.monotonic()
    ser = csv_handle = journal_handle = None
    pending = bytearray()
    pending_oversized = False

    def request_stop(_sig=None, _frame=None) -> None:
        nonlocal stop_requested
        metadata["termination"] = "user_interrupt"
        stop_requested = True

    def increment(key: str, amount: int = 1) -> None:
        metadata[key] = int(metadata.get(key, 0)) + amount

    def journal(kind: str, record: Dict[str, object]) -> None:
        if journal_handle is not None:
            journal_handle.write(json.dumps({"kind": kind, "record": record}, ensure_ascii=True) + "\n")

    def event(timestamp: str, message: str) -> None:
        record = {"Timestamp": timestamp, "Event": message}
        events.append(record)
        journal("event", record)

    def reject(timestamp: str, line: str, reason: str, parts: int) -> None:
        increment("bad_data_rows")
        if reason.startswith("field_count"):
            increment("bad_field_count_rows")
        elif reason.startswith("decode"):
            increment("bad_decode_rows")
        elif reason.startswith(("crc_", "oversized", "incomplete", "not_D")):
            increment("bad_crc_rows")
        else:
            increment("bad_numeric_rows")
        # Escaping retains corrupt/control characters without invalid XLSX XML.
        record = {"Timestamp": timestamp, "Chars": len(line), "Parts": parts,
                  "Reason": reason, "Raw Line": ascii(line)}
        if len(rejected_rows) < MAX_REJECTED_ROWS_IN_XLSX:
            rejected_rows.append(record)
        journal("rejected", record)
        count = int(metadata["bad_data_rows"])
        if count <= 10 or count % 100 == 0:
            print(f"[REJECT] #{count}: {reason}")
            if PRINT_REJECTED_RAW_ROWS:
                print(f"[RAW] {ascii(line)}")

    print(LOGGER_VERSION)
    print(f"[LOG] Serial: {SERIAL_PORT} @ {BAUD_RATE}")
    print(f"[LOG] CSV: {csv_path.resolve()}")
    print(f"[LOG] XLSX on shutdown: {xlsx_path.resolve()}")
    print("[LOG] Ctrl+C saves the workbook. ALARM does not stop logging.")
    previous_handler = signal.signal(signal.SIGINT, request_stop)
    try:
        csv_handle = csv_path.open("x", newline="", encoding="utf-8")
        journal_handle = journal_path.open("x", encoding="utf-8")
        writer = csv.DictWriter(csv_handle, fieldnames=CSV_FIELDS, extrasaction="ignore")
        writer.writeheader()
        journal("metadata", metadata)
        for handle in (csv_handle, journal_handle):
            handle.flush()
            os.fsync(handle.fileno())
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=SERIAL_TIMEOUT_S)
        while not stop_requested:
            # A timeout may return a partial line; retain it until a newline.
            # A size limit prevents unbounded allocation on noisy/no-newline input.
            raw = ser.readline(MAX_FRAME_BYTES + 1)
            now = time.monotonic()
            if now - last_flush >= CSV_FLUSH_INTERVAL_S:
                journal("metadata", metadata)
                for handle in (csv_handle, journal_handle):
                    handle.flush()
                last_flush = now
            if now - last_fsync >= CSV_FSYNC_INTERVAL_S:
                for handle in (csv_handle, journal_handle):
                    handle.flush()
                    os.fsync(handle.fileno())
                last_fsync = now
            if not raw:
                continue
            if len(pending) + len(raw) > MAX_FRAME_BYTES:
                pending_oversized = True
            pending.extend(raw[:max(0, MAX_FRAME_BYTES - len(pending))])
            if not raw.endswith(b"\n"):
                continue
            timestamp = datetime.now().isoformat(timespec="milliseconds")
            line = bytes(pending).decode("ascii", errors="replace").rstrip("\r\n")
            pending.clear()
            if pending_oversized:
                reject(timestamp, line, "oversized_frame", line.count(",") + 1)
                pending_oversized = False
                continue
            if not line:
                continue
            if any(ord(c) < 32 or ord(c) > 126 for c in line):
                reject(timestamp, line, "decode_corruption", line.count(",") + 1)
                continue
            firmware_match = RE_FIRMWARE.match(line)
            if firmware_match:
                metadata["firmware"] = firmware_match.group("name")
                journal("metadata", metadata)
                continue
            fields_match = RE_FIELDS.match(line)
            if fields_match:
                observed = fields_match.group("body").split(",")
                if observed != FIRMWARE_FIELDS:
                    event(timestamp, "FIRMWARE_FIELD_CONTRACT_MISMATCH")
                    if STRICT_FIELD_CONTRACT:
                        metadata["termination"] = "firmware_field_contract_mismatch"
                        exit_code = 1
                        stop_requested = True
                else:
                    metadata["schema_verified"] = True
                journal("metadata", metadata)
                continue
            config_match = RE_CONFIG.match(line)
            if config_match:
                for key, value in parse_config_pairs(config_match.group("body")):
                    if last_config.get(key) != value:
                        record = {"Timestamp": timestamp, "Key": key, "Value": value}
                        config_rows.append(record)
                        journal("config", record)
                        last_config[key] = value
                    if key == "SCHEMA_VERSION" and value != EXPECTED_SCHEMA_VERSION:
                        event(timestamp, "FIRMWARE_SCHEMA_VERSION_MISMATCH")
                        if STRICT_FIELD_CONTRACT:
                            metadata["termination"] = "firmware_schema_version_mismatch"
                            exit_code = 1
                            stop_requested = True
                continue
            event_match = RE_EVENT.match(line)
            if line.startswith("#"):
                event(timestamp, event_match.group("body") if event_match else line.lstrip("# "))
                print(line)
                continue

            row, reason, part_count = parse_data_line_diagnostic(line)
            if row is None:
                reject(timestamp, line, reason or "unknown_parse_failure", part_count)
                continue
            if previous_row is not None:
                delta = (int(row["telemetry_seq"]) - int(previous_row["telemetry_seq"])) & 0xFFFFFFFF
                dt = float(row["uptime_s"]) - float(previous_row["uptime_s"])
                # millis() wraps after ~49.7 days; sequence normally remains forward.
                uptime_wrap = float(previous_row["uptime_s"]) > 4_294_000 and float(row["uptime_s"]) < 1000
                if dt < 0 and not uptime_wrap:
                    increment("board_resets")
                    event(timestamp, "BOARD_RESET_OBSERVED: telemetry epoch restarted")
                elif delta == 0 or delta >= 0x80000000:
                    reject(timestamp, line, "duplicate_or_out_of_order_sequence", part_count)
                    continue
                else:
                    if delta > 1:
                        increment("missing_sequences", delta - 1)
                        event(timestamp, f"TELEMETRY_GAP: {delta - 1} missing emitted frame(s)")
                    if dt > 1.5:
                        event(timestamp, f"TELEMETRY_TIME_GAP: {dt:.3f} s; no values interpolated")
            previous_row = row
            if session_start is None:
                session_start = time.monotonic()
            elapsed = time.monotonic() - session_start
            row_out = {"Timestamp": timestamp, "Logger_elapsed_s": round(elapsed, 6),
                       "Logger_elapsed_h": round(elapsed / 3600, 9), **row}
            writer.writerow(row_out)
            increment("accepted_rows")
            if int(row["alarm"]):
                increment("alarm_rows")
            if int(row["control_gap_active"]):
                increment("control_gap_rows")
            state = SYSTEM_STATE_NAMES[int(row["sys_state"])]
            print(f"[DATA] {elapsed/3600:8.4f} h {state:6} "
                  f"CH1={fmt_number(row['ch1_C'])} C SP={fmt_number(row['ch1_setpoint_C'], precision=2)} "
                  f"duty={fmt_number(row['ch1_duty_pct'], precision=2)}% | "
                  f"CH2={fmt_number(row['ch2_C'])} C SP={fmt_number(row['ch2_setpoint_C'], precision=2)} "
                  f"duty={fmt_number(row['ch2_duty_pct'], precision=2)}% "
                  f"fault=0x{int(row['fault_flags']):04X}", flush=True)
    except (serial.SerialException, OSError) as exc:
        metadata["termination"] = f"acquisition_error: {exc}"
        exit_code = 1
        print(f"[ERROR] {metadata['termination']}")
    except KeyboardInterrupt:
        metadata["termination"] = "user_interrupt"
    finally:
        signal.signal(signal.SIGINT, previous_handler)
        if ser is not None:
            try:
                ser.close()
            except (OSError, serial.SerialException):
                pass
        try:
            if pending:
                reject(datetime.now().isoformat(timespec="milliseconds"),
                       bytes(pending).decode("ascii", errors="replace"),
                       "incomplete_frame_at_shutdown", 0)
            metadata["ended_at"] = datetime.now().isoformat(timespec="seconds")
            journal("metadata", metadata)
        except OSError as exc:
            print(f"[ERROR] Journal finalisation: {exc}")
            exit_code = 1
        for handle in (csv_handle, journal_handle):
            if handle is not None:
                try:
                    handle.flush()
                    os.fsync(handle.fileno())
                except OSError as exc:
                    print(f"[ERROR] Saving buffered data: {exc}")
                    exit_code = 1
                finally:
                    try:
                        handle.close()
                    except OSError:
                        exit_code = 1

    print(f"[LOG] Accepted: {metadata['accepted_rows']}; rejected: {metadata['bad_data_rows']}")
    if csv_path.exists():
        try:
            build_workbook(csv_path, xlsx_path, metadata, config_rows, events, rejected_rows)
            print(f"[SAVED] {xlsx_path.resolve()}")
        except Exception as exc:
            print(f"[ERROR] XLSX generation failed: {exc}")
            print(f"[LOG] Recover from existing CSV with --rebuild {csv_path}")
            exit_code = 1
    return exit_code


def rebuild(csv_path: Path) -> int:
    metadata = new_metadata()
    configs, events, rejected = [], [], []
    journal_path = journal_path_for(csv_path)
    if journal_path.exists():
        with journal_path.open(encoding="utf-8") as handle:
            for line in handle:
                try:
                    entry = json.loads(line)
                    kind, record = entry["kind"], entry["record"]
                    if kind == "metadata":
                        metadata.update(record)
                    elif kind == "config":
                        configs.append(record)
                    elif kind == "event":
                        events.append(record)
                    elif kind == "rejected" and len(rejected) < MAX_REJECTED_ROWS_IN_XLSX:
                        rejected.append(record)
                except (ValueError, TypeError, KeyError):
                    continue  # A power interruption may truncate the final journal line.
    metadata["accepted_rows"] = metadata["alarm_rows"] = metadata["control_gap_rows"] = 0
    first = last = None
    for row in iter_csv_rows(csv_path):
        first = first or row
        last = row
        metadata["accepted_rows"] += 1
        metadata["alarm_rows"] += int(row["alarm"])
        metadata["control_gap_rows"] += int(row["control_gap_active"])
    if first:
        metadata["started_at"] = first["Timestamp"]
        metadata["ended_at"] = last["Timestamp"]
    metadata["termination"] = "offline_rebuild; invalid/incomplete CSV rows skipped"
    xlsx_path = csv_path.with_suffix(".rebuilt.xlsx")
    number = 1
    while xlsx_path.exists():
        xlsx_path = csv_path.with_suffix(f".rebuilt_{number}.xlsx")
        number += 1
    build_workbook(csv_path, xlsx_path, metadata, configs, events, rejected)
    print(f"[SAVED] {xlsx_path.resolve()}")
    return 0


def main() -> int:
    global SERIAL_PORT, OUTPUT_DIRECTORY
    parser = argparse.ArgumentParser(description=LOGGER_VERSION)
    parser.add_argument("--port", default=SERIAL_PORT, help="serial port (default: COM4)")
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIRECTORY)
    parser.add_argument("--rebuild", type=Path, metavar="CSV", help="build XLSX from an existing V1.1 CSV without a serial connection")
    args = parser.parse_args()
    SERIAL_PORT = args.port
    OUTPUT_DIRECTORY = args.output_dir
    return rebuild(args.rebuild) if args.rebuild else run_logger()


if __name__ == "__main__":
    raise SystemExit(main())
