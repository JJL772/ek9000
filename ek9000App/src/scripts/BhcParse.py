#!/usr/bin/python3
# Parses a file lol
#
import sys, os, io
import CppTk, EpicsTk
import json, argparse, datetime

"""

JSON SYNTAX:


{
	"name": "EL3202",		// Name of the terminal
	"type": "AnalogIn",		// Type AnalogIn, AnalogOut, DigIn, DigOut
	"inputs": 2,			// Number of input channels
	"outputs": 0,			// Number of output channels
	"pdo_in_size": 4,		// PDO input size. For Digital terminals, this is COILS. For Analog terms, this is REGISTERS (16-bits per reg)  
	"pdo_out_size": 0		// PDO output size. Same rules as in_size.
},

"""

parser = argparse.ArgumentParser(description="Generates a header containing information about each individual terminal")
parser.add_argument("file", metavar="file", type=str, nargs=1, help="The file to parse for terminal info")
parser.add_argument("--output", "-o", dest="output", action='store', type=str, help="The name of the output header")
parser.add_argument("--verify", dest="verify", action="store_const", const=1, help="Verify the json too.")
parser.add_argument("--verbose", dest="verbose", action='store_true', help='Run with extra debug inf')
args = parser.parse_args()

#
# Files are in json format, check them out to figure out the actual syntax
#
file = args.file[0]
if not file or not os.path.exists(file):
	print("ERROR: Could not find input file.")
	sys.exit(1)

out = args.output
json_stuff = None
try:
	print("Parsing " + file)
	with open(file, "r") as fs:
		json_stuff = json.loads(fs.read())
except Exception as e:
	print("Error while parsing json:")
	print(str(e))
	sys.exit(1)

# Make a file to log the supported terminals in
fp = open("../../../SupportedDevices.md", "w")
fp.write("## Supported Devices\n\n")

#
# Header generation 
#
header = CppTk.Header(out)
header.add_block_comment("\nAUTOGENERATED FILE: DO NOT EDIT.\n")
header.fs.write("#pragma once\n\n")
header.include_std("stddef.h")
header.include_std("stdint.h")
header.newline()
header.begin_struct("terminal_s")
header.add_variable("m_pString", "const char*")
header.add_variable("m_nID", "uint32_t")
header.add_variable("m_nOutputSize", "uint16_t")
header.add_variable("m_nInputSize", "uint16_t")
header.end_struct()
header.add_typedef("terminal_t", "terminal_s")
header.add_typedef("STerminalInfoConst_t", "terminal_t")
header.newlines(3)

try:
	count = 0
	vars = list()
	for terminal in json_stuff["terminals"]:
		count = count + 1
		name = terminal["name"]
		fp.write("* " + name)
		fp.write("\n")
		vars.append("&" + name + "_Info")
		outsize = terminal["pdo_out_size"]
		insize = terminal["pdo_in_size"]
		header.add_block_comment(name)
		header.add_define(name + "_STRING", '"' + name + '"')
		header.add_define(name + "_ID", name.replace("EL", ""))
		header.add_define(name + "_OUTPUT_SIZE", str(outsize))
		header.add_define(name + "_INPUT_SIZE", str(insize))
		header.add_init_struct("STerminalInfoConst_t", name + "_Info", name + "_STRING", name + "_ID",
							   name + "_OUTPUT_SIZE", name + "_INPUT_SIZE", static=True, const=True)
	header.newlines(1)
	header.add_define("NUM_TERMINALS", str(count))
	header.newlines(1)
	header.add_array_variable("g_pTerminalInfos", "STerminalInfoConst_t*", vars, const=True, static=True)
except KeyError as e:
	print("Malformed JSON:")
	print("\tMissing the array key 'terminals'.")

fp.close()


class Terminal():
	def __init__(self, record:str, num:int, dtyp:str):
		self.vals = dict()
		self.record = record
		self.num = num
		self.vals['DTYP'] = dtyp

	# Adds a collection of values to the defaults list
	def add_values(self, _vals:dict):
		for v in _vals.items():
			self.vals[v[0]] = v[1]

	# Write this record out to fp
	def write(self, fp):
		for i in range(self.num):
			fp.write(f'record({self.record},"$(TERMINAL):{i+1}")\n{{\n')
			for i in self.vals.items():
				fp.write(f'\tfield({i[0]}, "{i[1]}")\n')
			fp.write('}\n\n')

	def set_default_bi(self):
		self.vals['ZNAM'] = 'low'
		self.vals['ONAM'] = 'high'
		self.vals['SCAN'] = '.1 second'
		self.vals['PINI'] = 'YES'

	def set_default_bo(self):
		self.vals['ZNAM'] = 'low'
		self.vals['ONAM'] = 'high'
		self.vals['SCAN'] = '.1 second'
		self.vals['PINI'] = 'YES'

	def set_default_ai(self):
		self.vals['LINR'] = 'LINEAR'
		self.vals['PINI'] = 'YES'
		self.vals['EGU'] = 'Volts'
		self.vals['SCAN'] = '.1 second'

	def set_default_ao(self):
		self.vals['LINR'] = 'LINEAR'
		self.vals['PINI'] = 'YES'
		self.vals['EGU'] = 'Volts'



#
# Template/subs generation
#
bi_record_template = \
	"""
record(bi,"$(TERMINAL):$$NUM")
{
        field(DESC,  "")
        field(DTYP,  "$$DTYP")
        field(ZNAM,  "low")
        field(ONAM,  "high")
        field(SCAN,  ".1 second")
        field(PINI,  "YES")
}
"""

bo_record_template = \
	"""
record(bo,"$(TERMINAL):$$NUM")
{
        field(DESC,  "")
        field(DTYP,  "$$DTYP")
        field(ZNAM,  "low")
        field(ONAM,  "high")
        field(PINI,  "YES")
}
"""

ai_record_template = \
	"""
record(ai,"$(TERMINAL):$$NUM")
{
        field(DESC,  "")
        field(DTYP,  "$$DTYP")
        field(SCAN,  ".1 second")
        field(EGU,   "Volts")
        field(LINR,  "LINEAR")
        field(PINI,  "YES")
}
"""

ao_record_template = \
	"""
record(ao,"$(TERMINAL):$$NUM")
{
        field(DESC,  "")
        field(DTYP,  "$$DTYP")
        field(SCAN,  ".1 second")
        field(LINR,  "LINEAR")
        field(PINI,  "YES")
}
"""

subs = \
	"""
file $$FILE
{       pattern
        {
                TERMINAL,

        }
        {
        }
}
"""


# Returns the dtyp of the terminal
def get_dtyp(terminal: dict) -> str:
	try:
		typ = terminal["dtyp"]
		return typ
	except:
		if terminal["type"] == "DigIn":
			return "EL10XX"
		elif terminal["type"] == "DigOut":
			return "EL20XX"
		elif terminal["type"] == "AnalogIn":
			return "EL30XX"
		elif terminal["type"] == "AnalogOut":
			return "EL40XX"
		else:
			raise ValueError(f"Type is {terminal['type']}")


with open("../../Db/Templates.mak", "w") as fp:
	print("Writing Templates.mak")
	fp.write("\n# AUTOGENERATED FILE, DO NOT EDIT\n\n")
	for terminal in json_stuff["terminals"]:
		template_filename = '../../Db/' + terminal["name"] + ".template"
		subs_filename = '../../Db/' + terminal["name"] + ".substitutions"
		name = terminal["name"]
		inputs = terminal["inputs"]
		outputs = terminal["outputs"]

		typ = get_dtyp(terminal)

		# Additional 'defaults'
		extras = {}
		try:
			extras = terminal['defaults']
		except:
			extras = {}

		tfp = open(template_filename, "w")
		sfp = open(subs_filename, "w")
		if terminal["type"] == "DigIn":
			t = Terminal('bi', inputs, typ)
			t.set_default_bi()
			t.add_values(extras)
			t.write(tfp)
		elif terminal["type"] == "DigOut":
			t = Terminal('bo', outputs, typ)
			t.set_default_bo()
			t.add_values(extras)
			t.write(tfp)
		elif terminal["type"] == "AnalogIn":
			t = Terminal('ai', inputs, typ)
			t.set_default_ai()
			t.add_values(extras)
			t.write(tfp)
		elif terminal["type"] == "AnalogOut":
			t = Terminal('ao', outputs, typ)
			t.set_default_ao()
			t.add_values(extras)
			t.write(tfp)
		# Write out the subs
		sfp.write(subs.replace("$$FILE", name + ".template"))
		fp.write("# {2} \nDB += {0} {1}\n\n".format(name + ".template", name + ".substitutions", name))
		tfp.close()
		sfp.close()
		if args.verbose:
			print("Wrote {0} and {1} (had {2} inputs and {3} outputs)".format(template_filename, subs_filename,
																			  inputs, outputs))
	fp.write("\n\n")
