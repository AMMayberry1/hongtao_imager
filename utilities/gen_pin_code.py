import csv
import argparse
import sys

class Pin:
    def __init__(self, bank, num, label, adc=None, adc_chan=None):
        self.bank     = bank.upper()
        self.num      = num
        self.label    = label.upper()
        self.adc      = adc
        self.adc_chan = adc_chan

        self.bank_define = self.label + "_BANK"
        self.num_define  = self.label + "_PIN"

        if self.adc != None:
            self.adc_define = self.label + "_ADC"
            self.adc_chan_define = self.label + "_ADC_CHAN"

    def __repr__(self):
        str_self = "P" + self.bank + self.num + ":" + " " + self.label
        
        if self.adc != None:
            str_self += "-ADC" + self.adc + " CH" + self.adc_chan

        return str_self

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("pin_list", help="CSV file containing pin list")
    parser.add_argument("out_file", help="file to dump text output to (.txt extension added automatically)")
    parser.add_argument("--is-adc", action="store_true", help="expect extra data related to ADC pins")
    args = parser.parse_args()

    in_filename = args.pin_list
    out_filename = args.out_file
    is_adc = args.is_adc

    try:
        in_file = open(in_filename, "rU")
    except IOError:
        print "Input file", in_filename, "could not be opened."
        sys.exit()

    try:
        out_file = open(out_filename + ".txt", "w")
    except IOError:
        print "Output file", out_filename + ".txt", "could not be opened."
        sys.exit()

    reader = csv.reader(in_file)
    reader.next() # skip first row

    pin_data = []
    for row in reader:
        if is_adc:
            pin_data.append(Pin(row[0][1], row[0][2:], row[1], row[2], row[3]))
        else:
            pin_data.append(Pin(row[0][1], row[0][2:], row[1]))
    in_file.close()

    pin_data.sort(key=lambda x: x.label)

    # print pin_data

    # pass 1: defines
    if is_adc:
        out_file.write("#define NUM_ADC_PINS " + str(len(pin_data)))
    else:
        out_file.write("#define NUM_MISC_PINS " + str(len(pin_data)))
    out_file.write("\n\n")

    for pin in pin_data:
        out_file.write("#define " + pin.bank_define + " GPIO" + pin.bank + "\n")
        out_file.write("#define " + pin.num_define + " GPIO_PIN_" + pin.num + "\n")

        if is_adc:
            out_file.write("#define " + pin.adc_define + " ADC" + pin.adc + "\n")
            out_file.write("#define " + pin.adc_chan_define + " ADC_CHANNEL_" + pin.adc_chan + "\n")


    out_file.write('\n')

    # pass 2: arrays
    if is_adc:
        out_file.write(build_array_text("GPIO_TypeDef *COL_ADC_BANK", [x.bank_define for x in pin_data]) + "\n\n")
        out_file.write(build_array_text("uint16_t COL_ADC_PIN", [x.num_define for x in pin_data]) + "\n\n")
        out_file.write(build_array_text("ADC_TypeDef *COL_ADC", [x.adc_define for x in pin_data]) + "\n\n")
        out_file.write(build_array_text("uint32_t COL_ADC_CHAN", [x.adc_chan_define for x in pin_data]) + "\n\n")
    else:
        out_file.write(build_array_text("GPIO_TypeDef *MISC_BANK", [x.bank_define for x in pin_data]) + "\n\n")
        out_file.write(build_array_text("uint16_t MISC_PIN", [x.num_define for x in pin_data]) + "\n\n")

def build_array_text(label, values):
    array_str = label + "[] = {" + values[0]

    for val in values[1:]:
        array_str += ", " + val

    array_str += "};"

    return array_str


main()