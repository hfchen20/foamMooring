with open("background/log.overInterDyMFoam", "r") as f:
    # Look for the ending orientation
    txt = f.read()
    start = txt.find('Orientation: (')
    assert start != -1
    txt = txt[start + len('Orientation: ('):]
    txt = txt[:txt.find(')')]
    values = [float(v) for v in txt.split(' ')]
    assert values[0] < 1 and values[2] > 0.05
