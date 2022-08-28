import sys, getopt, csv

def read_from_CSV(file_name, frequency, start_time, end_time):
    data = []
    with open(file_name, 'r', newline='') as csvfile:
        csv_reader = csv.reader(csvfile,quoting=csv.QUOTE_ALL)
        for idx, row in enumerate(csv_reader):
            if(idx % frequency == 0):
                if(row[1] >  start_time and row[1] < end_time):
                    data.append(row)
    csvfile.close()
    data.pop(0) # This is a timestamp as to when the data was erased last
    return data

def convert_row(row):
    return """
        <wpt lat="%s" lon="%s"><ele>0</ele><time>%sT%sZ</time><extensions><gpxtpx:TrackPointExtension><gpxtpx:hr>171</gpxtpx:hr></gpxtpx:TrackPointExtension></extensions></wpt>
    """ % (row[3], row[2], row[0], row[1])

def write_to_xml(data, inputfile):
    begin = """<?xml version="1.0" encoding="UTF-8"?>
    <gpx>
    """
    centre = [convert_row(row) for row in data]
    end = """
    </gpx>
    """
    final = begin + ''.join(centre) + end
    file = inputfile[0:len(inputfile) - 4] + '_'
    date = data[0][0]
    # for char in date:
    date = date.replace('/', '_')
    output_name = file + date + '.gpx'
    print(output_name)
    with open(output_name, 'w') as f:
        f.write(final)
    f.close()
    print('Succesfully wrote', output_name)

def main(argv):
    inputfile = ''
    start_time = "1:56:00" # +8 -> 9:56 AM
    end_time = "2:48:00" # +8 -> 10:48 AM
    frequency = 1 # every 1 second
    try:
        opts, args = getopt.getopt(argv,"hi:f:",["ifile=","freq="])
    except getopt.GetoptError:
        print('csvToGPX.py -i <inputfile> -f <frequency>')
        print('inputfile: the file to be parsed')
        print('frequency: use data for each second specified i.e. "-f 60" is every 60 seconds')
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print('csvToGPX.py -i <inputfile> -f <frequency>')
            print('inputfile: the file to be parsed')
            print('frequency: use data for each second specified i.e. "-f 60" is every 60 seconds')
            sys.exit()
        elif opt in ("-i", "--ifile"):
            inputfile = arg
        elif opt in ("-f", "--freq"):
            frequency = int(arg)

    data = read_from_CSV(inputfile, frequency, start_time, end_time)
    write_to_xml(data, inputfile)

if __name__=='__main__':
    main(sys.argv[1:])