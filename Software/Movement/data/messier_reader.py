#-------------------------------------------------------------------------------------------#
#csv
import csv
from skyfield.api import Star, load

ts = load.timescale()
t = ts.now()

messier = {}

with open('messier_qc.csv', newline='', encoding='utf-8') as f:
    reader = csv.DictReader(f)
    for row in reader:
        star = Star(
            ra_hours=float(row['ra_hours']),
            dec_degrees=float(row['dec_degrees'])
        )
        messier[row['name']] = {
            'type': row['type'],
            'mag': float(row['mag']),
            'comment': row['comment'],
            'star': star
        }

# Exemple : obtenir la position de M31
earth = load('de421.bsp')['earth']
m31 = messier['M31']['star']
astrometric = earth.at(t).observe(m31)
ra, dec, distance = astrometric.radec()

print("M31 RA:", ra)
print("M31 Dec:", dec)

#-------------------------------------------------------------------------------------------#
import json
from skyfield.api import Star, load

with open('messier_qc.json', 'r', encoding='utf-8') as f:
    data = json.load(f)

messier = {}

for obj in data["objects"]:
    messier[obj["name"]] = {
        "type": obj["type"],
        "mag": obj["mag"],
        "comment": obj["comment"],
        "star": Star(
            ra_hours=obj["ra_hours"],
            dec_degrees=obj["dec_degrees"]
        )
    }

# Exemple : position de M31
ts = load.timescale()
t = ts.now()
earth = load('de421.bsp')['earth']

m31 = messier["M31"]["star"]
astrometric = earth.at(t).observe(m31)
ra, dec, distance = astrometric.radec()

print("M31 RA:", ra)
print("M31 Dec:", dec)

#-------------------------------------------------------------------------------------------#
