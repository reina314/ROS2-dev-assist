#!/usr/bin/env python3

import os
import string
import random
import argparse
from merger import Merger


class Compiler():
    def __init__(self, program_filepath, verbose = False) -> None:
        print(f'[*] processing {program_filepath}...')

        self.program_filepath: str = program_filepath
        self.line_number: int = 0
        self.verbose: bool = verbose 

        self.data = {
            # Compile options
            'NODE'          : ''.join(random.choices(string.ascii_letters + string.digits, k=10)),
            'TAB'           : 4,
            'DESCRIPTION'   : '',

            # Communication
            'PUBLISHER' : [],
            'SUBSCRIBER': [],
            'CLIENT'    : [],
            'SERVER'    : [],
            'ACLIENT'   : [],
            'ASERVER'   : [],

            # Other objects
            'TIMER'     : [],

            # Message types
            'MSG'       : [],
            'SRV'       : [],
        }

        self.read()
        self.parse()
        self.write()
        self.merge()

        print(f'[*] {program_filepath} is successfully compiled and saved as {self.data["NODE"].lower()}.py!\n\n')

    
    def raiseError(self, msg: str) -> None:
        print(f"[!] line {self.line_number}: {msg}")
        exit()


    def read(self) -> None:
        print('[*] loading...')
        self.program_lines = []
        with open(self.program_filepath, 'r') as program_file:
            self.program_lines = [
                line for line in program_file.readlines()
            ]


    def parse(self) -> None:
        print('[*] parsing...')

        # parse the code line by line
        func = self.analyze_config
        for line in self.program_lines:
            self.line_number += 1
            has_indent = self.has_indent(line=line)
            line = self.remove_comment(line=line)

            if (not has_indent):
                if ('PUBLISHER:' in line):
                    func = self.analyze_pub
                    continue
                elif ('SUBSCRIBER:' in line):
                    func = self.analyze_sub
                    continue
                elif ('CLIENT:' in line):
                    func = self.analyze_cli
                    continue
                elif ('SERVER:' in line):
                    func = self.analyze_ser
                    continue
                elif ('TIMER:' in line):
                    func = self.analyze_timer
                    continue
                elif (':' in line):
                    func = self.analyze_config

            func(line=line)

        # delete duplicated types
        for msg_type in ['MSG', 'SRV']:
            unique_tuples = set(tuple(sublist) for sublist in self.data[msg_type])
            self.data[msg_type] = [list(tup) for tup in unique_tuples]

        if self.verbose: print(f'[+] parsed data: {self.data}')


    def has_indent(self, line: str) -> bool:
        return len(line) != len(line.lstrip())


    def remove_comment(self, line: str) -> str:
        index = line.find('#')
        return line if index == -1 else line[0:line.find('#') - 1].strip()


    def analyze_opcode(self, line: str) -> tuple[str, list[str]]:
        parsed_code = [x.strip() for x in line.split(':')]
        try:
            lvalue: str         = parsed_code[0]
            rvalue: list[str]   = parsed_code[1]
            return lvalue, rvalue
        except IndexError:
            return None, None


    def analyze_config(self, line: str) -> None:
        name, data = self.analyze_opcode(line=line)
        try:
            self.data[name.upper()] = int(data) if data.isdigit() else data
        except (AttributeError, IndexError):
            return


    def analyze_pub(self, line: str) -> None:
        name, data = self.analyze_opcode(line=line)
        if (data == None): return
        args: list[str] = [x.strip() for x in data.split(',')]
        
        try:
            name       = str(name)
            msg_type    = str(args[0])
            topic       = str(args[1])
        except IndexError:
            self.raiseError(f"wrong arguments for publisher: {line}")

        msg = [x.strip() for x in msg_type.split('/')]
        if (len(msg) < 2): 
            self.raiseError(f"wrong message type given: {msg[0]}")
        self.data['MSG'].append(msg)
        
        try:
            if args[2].isdigit(): 
                queue_size  = int(args[2])
            else:
                self.raiseError(f"queue_size must be integer: {args[2]}")
        except (IndexError, ValueError):
            queue_size  = 10 

        data = {
            'name'     : name,
            'type'      : msg_type,
            'topic'     : topic,
            'queue_size': queue_size
        }
        self.data['PUBLISHER'].append(data)


    def analyze_sub(self, line: str) -> None:
        name, data = self.analyze_opcode(line=line)
        if (data == None): return
        args: list[str] = [x.strip() for x in data.split(',')]
        
        try:
            name       = str(name)
            msg_type    = str(args[0])
            topic       = str(args[1])
            callback    = str(args[2])
        except IndexError:
            self.raiseError(f"wrong arguments for subscriber: {line}")
        
        msg = [x.strip() for x in msg_type.split('/')]
        if (len(msg) < 2): 
            self.raiseError(f"wrong message type given: {msg[0]}")
        self.data['MSG'].append(msg)

        try:
            if args[3].isdigit(): 
                queue_size  = int(args[3])
            else:
                self.raiseError(f"queue_size must be integer: {args[3]}")
        except (IndexError, ValueError):
            queue_size  = 10 

        data = {
            'name'     : name,
            'type'      : msg_type,
            'topic'     : topic,
            'callback'  : callback,
            'queue_size': queue_size
        }
        self.data['SUBSCRIBER'].append(data)


    def analyze_cli(self, line: str) -> None:
        pass


    def analyze_ser(self, line: str) -> None:
        pass


    def analyze_acl(self, line: str) -> None:
        pass


    def analyze_ase(self, line: str) -> None:
        pass


    def analyze_timer(self, line: str) -> None:
        name, data = self.analyze_opcode(line=line)
        if (data == None): return
        args: list[str] = [x.strip() for x in data.split(',')]
        
        try:
            name       = str(name)
            interval    = int(args[0])
            callback    = str(args[1])
        except IndexError:
            self.raiseError(f"wrong arguments for timer: {line}")

        data = {
            'name'     : name,
            'interval'  : interval,
            'callback'  : callback
        }
        self.data['TIMER'].append(data)


    def tab(self, number: int = 1) -> str:
        tab = ''
        for _ in range(int(self.data['TAB']) * number):
            tab += ' '
        return tab


    def write(self) -> None:
        print('[*] compiling...')
        with open(f'.{self.data["NODE"].lower()}.py.tmp', 'w') as output_file:
            output_file.write(
                'import rclpy\n'
                'from rclpy.node import Node\n'
            )

            # import necessary msg types
            for msg_type in self.data['MSG']:
                output_file.write(f'from {msg_type[0]}.msgs import {msg_type[1]}\n')

            # import necessary srv types
            for msg_type in self.data['SRV']:
                output_file.write(f'from {msg_type[0]}.srvs import {msg_type[1]}\n')

            output_file.write(
                f'\n\nclass {self.data["NODE"][0].upper() + self.data["NODE"][1:].lower()}(Node):\n'
                f'{self.tab()}def __init__(self) -> None:\n'
                f'{self.tab(2)}"""\n'
                f'{self.tab(2)}{self.data["DESCRIPTION"]}\n'
                f'{self.tab(2)}"""\n'
                f'{self.tab(2)}super().__init__("{self.data['NODE']}")\n'
            )

            # publisher
            if (len(self.data['PUBLISHER']) > 0):
                output_file.write(f'\n{self.tab(2)}# Publisher\n')
                for pub in self.data['PUBLISHER']:
                    output_file.write(f'{self.tab(2)}self.{pub["name"]} = self.create_publisher({pub["type"].split("/")[1]}, "{pub['topic']}", {pub["queue_size"]})\n')

            # subscriber
            if (len(self.data['SUBSCRIBER']) > 0):
                output_file.write(f'\n{self.tab(2)}# Subscriber\n')
                for sub in self.data['SUBSCRIBER']:
                    output_file.write(f'{self.tab(2)}self.{sub["name"]} = self.create_subscription({sub["type"].split("/")[1]}, "{sub['topic']}", self.{sub["callback"]}, {sub["queue_size"]})\n')

            # timer
            if (len(self.data['TIMER']) > 0):
                output_file.write(f'\n{self.tab(2)}# Timer\n')
                for timer in self.data['TIMER']:
                    output_file.write(f'{self.tab(2)}self.{timer["name"]} = self.create_timer({timer["interval"]}, self.{timer["callback"]})\n')

            output_file.write('\n\n')

            # callback
            if (len(self.data['SUBSCRIBER']) > 0):
                output_file.write(f'\n{self.tab()}# Callback')
                for sub in self.data['SUBSCRIBER']:
                    output_file.write(
                        f'\n{self.tab()}def {sub["callback"]}(self) -> None:\n'
                        f'{self.tab(2)}pass\n'
                    )

            # main
            output_file.write(
                '\n\ndef main(args=None):\n'
                f'{self.tab()}rclpy.init(args=args)\n'
                f'\n{self.tab()}node = {self.data["NODE"][0].upper() + self.data["NODE"][1:].lower()}()\n'
                f'{self.tab()}rclpy.spin(node)\n'
                f'\n{self.tab()}node.destroy_node()\n'
                f'{self.tab()}rclpy.shutdown()\n'
            )


    def merge(self) -> None:
        Merger(f'{self.data["NODE"].lower()}.py', f'.{self.data["NODE"].lower()}.py.tmp', f'{self.data["NODE"].lower()}.py', self.verbose)

        try:
            os.remove(f'.{self.data["NODE"].lower()}.py.tmp')
        except Exception as e:
            self.raiseError(f"can't delete .{self.data['NODE'].lower()}.py.tmp: {e}")


def main():
    parser = argparse.ArgumentParser(description="")
    parser.add_argument("args", nargs='+', help="Arguments (one or more).")
    parser.add_argument("--verbose", action="store_true", help="Increase output verbosity.")
    args = parser.parse_args()

    for arg in args.args:
        Compiler(arg, args.verbose)


if __name__ == '__main__':
    main()
