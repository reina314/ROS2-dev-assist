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
        self.line: str = ''
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
            self.line = self.remove_comment(line=line)
            name, data = self.analyze_opcode(line=self.line)

            if (not has_indent):
                if (name.upper() == 'PUBLISHER'):
                    func = self.analyze_pub
                    continue
                elif (name.upper() == 'SUBSCRIBER'):
                    func = self.analyze_sub
                    continue
                elif (name.upper() == 'CLIENT'):
                    func = self.analyze_cli
                    continue
                elif (name.upper() == 'SERVER'):
                    func = self.analyze_ser
                    continue
                elif (name.upper() == 'TIMER'):
                    func = self.analyze_timer
                    continue
                elif (name != None and data != None):
                    func = self.analyze_config

            func(name, data)

        # delete duplicated types
        for msg_type in ['MSG', 'SRV']:
            unique_tuples = set(tuple(sublist) for sublist in self.data[msg_type])
            self.data[msg_type] = [list(tup) for tup in unique_tuples]

        if self.verbose: print(f'[+] parsed data: {self.data}')


    def has_indent(self, line: str) -> bool:
        return len(line) != len(line.lstrip())


    def isfloat(self, data: str) -> bool:
        try:
            data = float(data)
            return True
        except ValueError:
            return False


    def remove_comment(self, line: str) -> str:
        index = line.find('#')
        return line if index == -1 else line[0:line.find('#') - 1].strip()


    def analyze_opcode(self, line: str) -> tuple[str, list[str]]:
        parsed_code = [x.strip() for x in line.split(':')]
        try:
            name: str         = parsed_code[0]
            data: list[str]   = parsed_code[1]
            return name, data
        except IndexError:
            return name, None


    def analyze_config(self, name: str, data: list[str]) -> None:
        try:
            if data.isdecimal():
                self.data[name.upper()] = int(data)
            elif self.isfloat(data):
                self.data[name.upper()] = float(data)
            else:
                self.data[name.upper()] = data
        except (AttributeError, IndexError):
            return


    def analyze_pub(self, name: str, data: list[str]) -> None:
        if (data == None): return
        args: list[str] = [x.strip() for x in data.split(',')]
        
        try:
            name        = str(name)
            msg_type    = str(args[0])
            topic       = str(args[1])
        except IndexError:
            self.raiseError(f"wrong arguments for publisher: {self.line}")

        msg = [x.strip() for x in msg_type.split('/')]
        if (len(msg) < 2): 
            self.raiseError(f"wrong message type given: {msg[0]}")
        self.data['MSG'].append(msg)
        
        try:
            if args[2].isdecimal(): 
                queue_size  = int(args[2])
            else:
                self.raiseError(f"queue_size must be integer: {args[2]}")
        except (IndexError, ValueError):
            queue_size  = 10 

        data = {
            'name'      : name,
            'type'      : msg_type,
            'topic'     : topic,
            'queue_size': queue_size
        }
        self.data['PUBLISHER'].append(data)


    def analyze_sub(self, name: str, data: list[str]) -> None:
        if (data == None): return
        args: list[str] = [x.strip() for x in data.split(',')]
        
        try:
            name        = str(name)
            msg_type    = str(args[0])
            topic       = str(args[1])
            callback    = str(args[2])
        except IndexError:
            self.raiseError(f"wrong arguments for subscriber: {self.line}")
        
        msg = [x.strip() for x in msg_type.split('/')]
        if (len(msg) < 2): 
            self.raiseError(f"wrong message type given: {msg[0]}")
        self.data['MSG'].append(msg)

        try:
            if args[3].isdecimal(): 
                queue_size  = int(args[3])
            else:
                self.raiseError(f"queue_size must be integer: {args[3]}")
        except (IndexError, ValueError):
            queue_size  = 10 

        data = {
            'name'      : name,
            'type'      : msg_type,
            'topic'     : topic,
            'callback'  : callback,
            'queue_size': queue_size
        }
        self.data['SUBSCRIBER'].append(data)


    def analyze_cli(self, name: str, data: list[str]) -> None:
        if (data == None): return
        args: list[str] = [x.strip() for x in data.split(',')]
        
        try:
            name       = str(name)
            srv_type    = str(args[0])
            service       = str(args[1])
        except IndexError:
            self.raiseError(f"wrong arguments for client: {self.line}")

        msg = [x.strip() for x in srv_type.split('/')]
        if (len(msg) < 2): 
            self.raiseError(f"wrong message type given: {msg[0]}")
        self.data['SRV'].append(msg)
        
        try:
            if self.isfloat(args[2]): 
                timeout  = float(args[2])
            else:
                self.raiseError(f"timeout must be float: {args[2]}")
        except (IndexError, ValueError):
            timeout  = 1.0 

        data = {
            'name'      : name,
            'type'      : srv_type,
            'service'   : service,
            'timeout'   : timeout
        }
        self.data['CLIENT'].append(data)


    def analyze_ser(self, name: str, data: list[str]) -> None:
        pass


    def analyze_acl(self, name: str, data: list[str]) -> None:
        pass


    def analyze_ase(self, name: str, data: list[str]) -> None:
        pass


    def analyze_timer(self, name: str, data: list[str]) -> None:
        if (data == None): return
        args: list[str] = [x.strip() for x in data.split(',')]
        
        try:
            name       = str(name)
            interval    = int(args[0])
            callback    = str(args[1])
        except IndexError:
            self.raiseError(f"wrong arguments for timer: {self.line}")

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
                output_file.write(f'from {msg_type[0]}.msg import {msg_type[1]}\n')

            # import necessary srv types
            for msg_type in self.data['SRV']:
                output_file.write(f'from {msg_type[0]}.srv import {msg_type[1]}\n')

            output_file.write(
                f'\n\nclass {self.data["NODE"][0].upper() + self.data["NODE"][1:].lower()}(Node):\n'
                f'{self.tab()}def __init__(self) -> None:\n'
                f'{self.tab(2)}"""\n'
                f'{self.tab(2)}{self.data["DESCRIPTION"]}\n'
                f'{self.tab(2)}"""\n'
                f'{self.tab(2)}super().__init__("{self.data["NODE"]}")\n'
            )

            # publisher
            if (len(self.data['PUBLISHER']) > 0):
                output_file.write(f'\n{self.tab(2)}# Publisher\n')
                for pub in self.data['PUBLISHER']:
                    output_file.write(f'{self.tab(2)}self.{pub["name"]} = self.create_publisher({pub["type"].split("/")[1]}, "{pub["topic"]}", {pub["queue_size"]})\n')

            # subscriber
            if (len(self.data['SUBSCRIBER']) > 0):
                output_file.write(f'\n{self.tab(2)}# Subscriber\n')
                for sub in self.data['SUBSCRIBER']:
                    output_file.write(f'{self.tab(2)}self.{sub["name"]} = self.create_subscription({sub["type"].split("/")[1]}, "{sub["topic"]}", self.{sub["callback"]}, {sub["queue_size"]})\n')

            # client
            if (len(self.data['CLIENT']) > 0):
                output_file.write(f'\n{self.tab(2)}# Client\n')
                for cli in self.data['CLIENT']:
                    output_file.write(f'{self.tab(2)}self.{cli["name"]} = self.create_client({cli["type"].split("/")[1]}, "{cli["service"]}")\n')

            # timer
            if (len(self.data['TIMER']) > 0):
                output_file.write(f'\n{self.tab(2)}# Timer\n')
                for timer in self.data['TIMER']:
                    output_file.write(f'{self.tab(2)}self.{timer["name"]} = self.create_timer({timer["interval"]}, self.{timer["callback"]})\n')

            # wait for service
            if (len(self.data['CLIENT']) > 0):
                output_file.write(f'\n{self.tab(2)}# wait for service\n')
                for cli in self.data['CLIENT']:
                    output_file.write(
                        f'{self.tab(2)}while not self.{cli["name"]}.wait_for_service(timeout_sec={cli["timeout"]}):\n'
                        f'{self.tab(3)}self.get_logger().info("service {cli["service"]} not avilable, waiting...")\n'
                    )

            output_file.write('\n\n')

            # callback
            if (len(self.data['SUBSCRIBER']) > 0):
                output_file.write(f'\n{self.tab()}# Callback for Subscriber')
                for sub in self.data['SUBSCRIBER']:
                    output_file.write(
                        f'\n{self.tab()}def {sub["callback"]}(self) -> None:\n'
                        f'{self.tab(2)}pass\n'
                    )
            if (len(self.data['TIMER']) > 0):
                output_file.write(f'\n{self.tab()}# Callback for Timer')
                for timer in self.data['TIMER']:
                    output_file.write(
                        f'\n{self.tab()}def {timer["callback"]}(self) -> None:\n'
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
    parser.add_argument("filepath", nargs='+', help="one or more yaml files' paths to compile")
    parser.add_argument("--verbose", action="store_true", help="increase output verbosity")
    args = parser.parse_args()

    for path in args.filepath:
        Compiler(path, args.verbose)


if __name__ == '__main__':
    main()
