# 2026 Science Olympiad Electric Vehicle
## 10th Place at State Competition

This is my partner's and my electric vehicle for the Science Olympiad at Harvard SciOly and States SciOly. The Harvard version is a Front Wheel Drive, straight-line robot using a goBILDA Super Speed Servo to drive it. The attempted States version uses two wheels driven by two Super Speed Servos in addition to motor encoders for the velocity PID and the goBILDA Odometry setup for the position PID.

---

## How to Use

Harvard version: To use this vehicle, align the starting wheels with the long edge of the starting tape using the included alignment tool. Before removing the alignment tool, ensure that the micro limit switch is positioned directly behind the bumper at the back (i.e. it will not activate right when you start the robot). Remove the alignment tool and use a #2 pencil to click the button on the breadboard to start the vehicle.

States version: This version is unfinished and untested, but ideally, you'd use a measuring tool to angle the robot at a starting heading. **The starting position and heading are really important - any errors in these will offset the rest of the run.** Press the button and the robot should start.

## Why I Made This

Since I was a kid, I've always loved engineering, coding, and building things with my hands. As I've got more into building electronics, I wanted to try my hand at this build event for this year's Science Olympiad season. This robot is for my third SciOly season, and I wanted to spend time to build something custom and good this year.

Harvard:

<img width="500" alt="Screenshot 2026-03-16 at 7 50 26 PM" src="https://github.com/user-attachments/assets/97eba9dd-decc-4143-b366-3677a4441586" />

States:

<img height="500" alt="Screenshot 2026-03-16 at 7 50 26 PM" src="https://github.com/user-attachments/assets/96494057-8905-4624-b505-28b8a22e17ff" />

## 3D CAD Model

I used Onshape to design this electric vehicle. You can find the Onshape design at this [link](https://cad.onshape.com/documents/1f13c2afbaef7ea2c9c90426/w/eae7389462c6562ac928b722/e/d476c01702c9b15c2478fb6e), with a versioned history of Harvard and States iterations. All my CAD files are in the 'cad' folder.

Harvard:

<img width="500" alt="Screenshot 2026-03-16 at 7 52 25 PM" src="https://github.com/user-attachments/assets/5e877def-e95b-47a9-b960-823b912f2ae8" />

States:

<img width="500" alt="Screenshot 2026-03-16 at 7 50 26 PM" src="https://github.com/user-attachments/assets/ae73e742-7c04-40d2-9c6a-784300e947bf" />

## Wiring

This robot does not use a PCB and uses jumper wires + a breadboard for its wiring. The wiring schematic is below:

Harvard:

<img width="700" alt="Screenshot 2026-03-16 at 7 53 42 PM" src="https://github.com/user-attachments/assets/59bdebac-6d94-4164-9438-47f8864ada25" />

-- states --

## Programming

This robot should be programmed through the Arduino IDE as it uses an Arduino Uno. Please install the library in the 'firmware/libraries' folder into the Arduino IDE (instructions can be found online). **Do not install the goBILDA Pinpoint library typically found in the Arduino library search, as that is not compatible with the Arduino Uno. Use the included library in the firmware/libraries folder instead.** Note: The Harvard robot's code works fine, but the attempted States code does not work yet. You will need to re-tune the PID and/or re-code it... it failed for me.

## Bill of Materials (BOM) for V1 (Harvard)

| **Name** | **Quantity** | **Price (per part)** | **Link** |
| --- | --- | --- | --- |
| 90mm REV Traction Wheel | 4   | $6.25 | [https://www.revrobotics.com/DUO-Traction-Wheels/](https://www.revrobotics.com/DUO-Traction-Wheels/) |
| 30T 5mm Plastic Bore REV Gear | 1   | $1.25 | [https://www.revrobotics.com/5mm-Plastic-Hex-Bore-Gears/](https://www.revrobotics.com/5mm-Plastic-Hex-Bore-Gears/) |
| 30T goBILDA Brass Servo Gear | 1   | $10.99 | [https://www.gobilda.com/2305-series-brass-mod-0-8-servo-gear-25-tooth-spline-30-tooth/](https://www.gobilda.com/2305-series-brass-mod-0-8-servo-gear-25-tooth-spline-30-tooth/) |
| 135mm (Length) 5mm (Width) REV Hex Shaft | 2   | $4.38 | [https://www.revrobotics.com/5mm-Hex-Shafts/](https://www.revrobotics.com/5mm-Hex-Shafts/) |
| REV Long Through Bore Bearing | 4   | $0.65 | [https://www.revrobotics.com/Through-Bore-Bearings/](https://www.revrobotics.com/Through-Bore-Bearings/) |
| REV Shaft Collar | 4   | $0.975 | [https://www.revrobotics.com/rev-41-1327-pk10/](https://www.revrobotics.com/rev-41-1327-pk10/) |
| HiLetgo Micro Limit Switch KW12-3 | 1   | $0.60 | [https://www.amazon.com/HiLetgo-KW12-3-Roller-Switch-Normally/dp/B07X142VGC](https://www.amazon.com/HiLetgo-KW12-3-Roller-Switch-Normally/dp/B07X142VGC) |
| REV 39mm Tensioning Bushing | 4   | $1.50 | [https://www.revrobotics.com/rev-41-1702/](https://www.revrobotics.com/rev-41-1702/) |
| goBILDA Super Speed Servo | 1   | $36.99 | [https://www.gobilda.com/2000-series-dual-mode-servo-25-4-super-speed/](https://www.gobilda.com/2000-series-dual-mode-servo-25-4-super-speed/) |
| Arduino Uno R3 | 1   | $14.99 | [https://www.amazon.com/ELEGOO-Board-ATmega328P-ATMEGA16U2-Compliant/dp/B01EWOE0UU/](https://www.amazon.com/ELEGOO-Board-ATmega328P-ATMEGA16U2-Compliant/dp/B01EWOE0UU/) |
| 9V Battery Cable | 1   | $0.50 | [https://www.amazon.com/Battery-Connector-Plastic-Experiment-Equipment/dp/B08SL9X2YC/](https://www.amazon.com/Battery-Connector-Plastic-Experiment-Equipment/dp/B08SL9X2YC/) |
| 9V Alkaline Battery | 1   | $1.59 | [https://www.amazon.com/Battery-Connector-Plastic-Experiment-Equipment/dp/B08SL9X2YC/](https://www.amazon.com/Battery-Connector-Plastic-Experiment-Equipment/dp/B08SL9X2YC/) |
| ¼” Wooden Dowel | 1   | $0.12 | [https://www.amazon.com/Dowel-Rods-Natural-Projects-Hardwood/dp/B0CR7J7YBJ/](https://www.amazon.com/Dowel-Rods-Natural-Projects-Hardwood/dp/B0CR7J7YBJ/) |
| **Total** |     |     | **$80.79** |

## Bill of Materials (BOM) for V2 (Attempted States)

| **Name** | **Quantity** | **Price (per part)** | **Link** |
| --- | --- | --- | --- |
| goBILDA 3607 Series Disc Wheel (14mm Bore, 120mm Diameter, Black) | 2   | $4.49 | [https://www.gobilda.com/3607-series-disc-wheel-14mm-bore-120mm-diameter-black-2-pack/](https://www.gobilda.com/4-bar-odometry-pack-2-pods-1-pinpoint-computer/) |
| goBILDA 4-Bar Odometry Pack (2 Pods, 1 Pinpoint Computer) | 1  | $279.99 | [https://www.gobilda.com/4-bar-odometry-pack-2-pods-1-pinpoint-computer/](https://www.gobilda.com/4-bar-odometry-pack-2-pods-1-pinpoint-computer/) |
| goBILDA 1309 Series Sonic Hub (8mm REX® Bore) | 4  | $7.99 | [https://www.gobilda.com/4-bar-odometry-pack-2-pods-1-pinpoint-computer/\](https://www.gobilda.com/4-bar-odometry-pack-2-pods-1-pinpoint-computer/](<https://www.gobilda.com/4-bar-odometry-pack-2-pods-1-pinpoint-computer/\](https://www.gobilda.com/4-bar-odometry-pack-2-pods-1-pinpoint-computer/>) |
| goBILDA 8mm REX® Shaft with E-Clip (Stainless Steel, 40mm Length) | 2  | $4.09 | [https://www.gobilda.com/8mm-rex-shaft-with-e-clip-stainless-steel-40mm-length/](https://www.gobilda.com/8mm-rex-shaft-with-e-clip-stainless-steel-40mm-length/) |
| M4 Nuts and Bolts | 29 (# of bolts, buy an equal amount of nuts)  | $0.03 | [https://www.amazon.com/VGBUY-312Pcs-Screws-25-50mm-Assortment/dp/B0DD7D6VP5/](https://www.amazon.com/VGBUY-312Pcs-Screws-25-50mm-Assortment/dp/B0DD7D6VP5/) |
| M3 Nuts and Bolts | 4 (# of bolts, buy an equal amount of nuts)  | $0.02 | [https://www.amazon.com/VGBUY-312Pcs-Screws-25-50mm-Assortment/dp/B0D14BC8QS/](https://www.amazon.com/VGBUY-312Pcs-Screws-25-50mm-Assortment/dp/B0D14BC8QS/) |
| Arduino Uno R3 | 1   | $14.99 | [https://www.amazon.com/ELEGOO-Board-ATmega328P-ATMEGA16U2-Compliant/dp/B01EWOE0UU/](https://www.amazon.com/ELEGOO-Board-ATmega328P-ATMEGA16U2-Compliant/dp/B01EWOE0UU/) |
| 30T goBILDA Brass Servo Gear | 1   | $10.99 | [https://www.gobilda.com/2305-series-brass-mod-0-8-servo-gear-25-tooth-spline-30-tooth/](https://www.gobilda.com/2305-series-brass-mod-0-8-servo-gear-25-tooth-spline-30-tooth/) |
| goBILDA Super Speed Servo | 2   | $36.99 | [https://www.gobilda.com/2000-series-dual-mode-servo-25-4-super-speed/](https://www.gobilda.com/2000-series-dual-mode-servo-25-4-super-speed/) |
| REV 39mm Tensioning Bushing | 4   | $1.50 | [https://www.revrobotics.com/rev-41-1702/](https://www.revrobotics.com/rev-41-1702/) |
| 18mmx9mm Bubble Level | 1   | $0.95 | [https://www.aliexpress.us/item/3256808611329580.html](https://www.aliexpress.us/item/3256808611329580.html) |
| Steel Caster Wheels | 1   | $2.50 | [https://www.amazon.com/dp/B0C69FYKC3/](https://www.amazon.com/dp/B0C69FYKC3/) |
| DC Motor Rotary Quadrature Encoder 600P/R | 2   | $18.99 | [https://www.amazon.com/dp/B07MX1SYXB/](https://www.amazon.com/dp/B07MX1SYXB/) |
| AA Battery Holder | 1   | $4.50 | [https://www.amazon.com/Thicken-Battery-Holder-Standard-Connector/dp/B07WP1CYYW/](https://www.amazon.com/Thicken-Battery-Holder-Standard-Connector/dp/B07WP1CYYW/) |
| AA NiMH Batteries | 8   | $1.60 | [https://www.amazon.com/AmazonBasics-Rechargeable-Batteries-8-Pack-Pre-charged/dp/B00CWNMV4G/](https://www.amazon.com/AmazonBasics-Rechargeable-Batteries-8-Pack-Pre-charged/dp/B00CWNMV4G/) |
| ¼” Wooden Dowel | 1   | $0.12 | [https://www.amazon.com/Dowel-Rods-Natural-Projects-Hardwood/dp/B0CR7J7YBJ/](https://www.amazon.com/Dowel-Rods-Natural-Projects-Hardwood/dp/B0CR7J7YBJ/) |
| **Total** |     |     | **$476.83** |
