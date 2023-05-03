
DIGIT_LOOKUP = {
    153: 3,
    94: 6,
    93: 0,
    62: 2,
    133: 1,
    169: 8,
    200: 4,
    201: 7,
    58: 5,
    162: 9,
}


def read_dict():
    labels = {}
    with open('log.txt', 'r') as log_reader:
        lines = log_reader.readlines()
        for line in lines:
            words = line.split(':')
            seq, tagid = int(words[0]), int(words[1])
            if tagid != 77:
                labels[seq] = DIGIT_LOOKUP[tagid]
    return labels