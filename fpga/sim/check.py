"""Compile three image-processing modules and exercise the histogram equalizer."""
from pathlib import Path
import os
import subprocess
import tempfile

FPGA = Path(__file__).resolve().parents[1]
RTL = FPGA / 'rtl'
SIM = FPGA / 'sim'
IVERILOG = os.environ.get('IVERILOG', 'iverilog')
VVP = os.environ.get('VVP', 'vvp')


def main():
    with tempfile.TemporaryDirectory(prefix='tri_rtl_', dir=FPGA) as build:
        out = Path(build)
        for top, source in [
            ('gaussian_gray4', 'gaussian_gray4_bram_sp.sv'),
            ('quad_board_raw10_gbrg_half_gray', 'quad_board_raw10_gbrg_half_gray.sv'),
        ]:
            subprocess.run([IVERILOG, '-g2012', '-s', top, '-o',
                            str(out / f'{top}.vvp'), str(RTL / source)], check=True)
        for width, height, seed in [(16, 12, 1), (9, 7, 171), (32, 24, 983),
                                    (1, 1, 27), (1920, 4, 1003)]:
            compiled = out / 'tb_tone.vvp'
            subprocess.run([IVERILOG, '-g2012', '-s', 'tb_tone',
                            f'-Ptb_tone.W={width}', f'-Ptb_tone.H={height}',
                            f'-Ptb_tone.SEED={seed}', '-o', str(compiled),
                            str(SIM / 'original.sv'),
                            str(RTL / 'histogram_lane_bank_bram_mux.sv'),
                            str(RTL / 'histogram_equalize4_pipeline_r1.sv'),
                            str(SIM / 'tb_tone.sv')], check=True)
            result = subprocess.run([VVP, str(compiled)], text=True,
                                    capture_output=True, timeout=120, check=True)
            if 'TONE_PASS' not in result.stdout:
                raise RuntimeError(result.stderr or result.stdout)
            print(f'{width}x{height}: {result.stdout.splitlines()[0]}')


if __name__ == '__main__':
    main()
