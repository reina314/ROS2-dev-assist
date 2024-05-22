#!/opt/homebrew/bin/python3

import difflib
import argparse


class Merger():
    def __init__(self, original_path: str, incoming_path: str, output_path: str, verbose = False) -> None:
        print(f'[*] merge: {incoming_path} + {original_path} -> {output_path}')

        self.original_filepath: str = original_path
        self.incoming_filepath: str = incoming_path
        self.output_filepath: str   = output_path

        self.verbose: bool = verbose

        self.original_lines = []
        self.incoming_lines = []
        self.merged_lines   = []

        self.read()
        self.merge()
        self.write()

        print(f'\n[*] merge is successfully completed and saved as {output_path}!')


    def raiseError(self, msg: str) -> None:
        print(f"[!] {msg}")
        exit()


    def read(self) -> None:
        print(f'\n[*] reading {self.original_filepath}...')
        try:
            with open(self.original_filepath, 'r') as original_file:
                self.original_lines = original_file.readlines()
        except Exception as e:
            with open(self.original_filepath, 'w') as original_file:
                if self.verbose: print(f'[i] new file {self.original_filepath} created') 

        print(f'[*] reading {self.incoming_filepath}...')
        try:
            with open(self.incoming_filepath, 'r') as incoming_file:
                self.incoming_lines = incoming_file.readlines()
        except Exception as e:
            self.raiseError(f'failed to load {self.incoming_filepath}: {e}')


    def merge(self) -> None:
        print('[*] merging...')
        d = difflib.Differ()
        diff = list(d.compare(self.original_lines, self.incoming_lines))

        if self.verbose:
            for line in diff:
                print(f'[i] {line}')

        # Configure here to control the output
        for line in diff:
            if line.startswith('  '):    # No change, add
                self.merged_lines.append(line[2:])
            elif line.startswith('- '):  # Original line deleted in incoming file, add
                self.merged_lines.append(line[2:])
            elif line.startswith('+ '):  # New line added in incoming file, add
                self.merged_lines.append(line[2:])
            elif line.startswith('? '):  # Metadata about the change, ignore
                continue


    def write(self) -> None:
        print(f'[*] writing to {self.output_filepath}...')
        with open(self.output_filepath, 'w') as output_file:
            output_file.writelines(self.merged_lines)


def main():
    parser = argparse.ArgumentParser(description="")
    parser.add_argument("dest", nargs=1, help="Arguments (output).")
    parser.add_argument("srcs", nargs=2, help="Arguments (original and incoming).")
    parser.add_argument("--verbose", action="store_true", help="Increase output verbosity.")
    args = parser.parse_args()

    Merger(args.srcs[0], args.srcs[1], args.dest[0], args.verbose)


if __name__ == '__main__':
    main()
