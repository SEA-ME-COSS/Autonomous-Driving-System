import QtQuick 2.2
import QtQuick.Window 2.1
import QtQuick.Controls 1.4
import QtQuick.Controls.Styles 1.4
import QtQuick.Extras 1.4

Window {
    id: root
    title: "Head Unit"
    width: 1024
    height: 600
    visible: true

    FontLoader {
        id: font
        source: "../font/Nebula-Regular.otf"
    }

    ValueSource {
        id: valueSource
    }

    Item {
        id: container
        scale: 0.5
        width: 1024
        height: 600
        anchors.centerIn: parent

        //==================================================//
        //                    Background                    //
        //==================================================//

        Rectangle {
            id: background
            width: 600
            height: 1024
            anchors.centerIn: parent
            rotation: 90
            gradient: Gradient {
                GradientStop { position: 0.0; color: valueSource.light }
                GradientStop { position: 0.4; color: "white" }
                GradientStop { position: 0.6; color: "white" }
                GradientStop { position: 1.0; color: valueSource.light }
            }
        }

        //==================================================//
        //                    Gear Mode                     //
        //==================================================//

        Rectangle {  // P
            width: 130
            height: 130
            x: 20
            y: parent.height / 2 - height / 2 - 210
            color: "black"
            radius: 30

            Rectangle {
                width: 115
                height: 115
                anchors.centerIn: parent
                color: "white"
                radius: 22

                Text {
                    text: "P"
                    font.family: font.name
                    font.pixelSize: 130
                    color: "#555555"
                    x: 13
                    y: -20
                }
            }
        }

        Rectangle {  // R
            width: 130
            height: 130
            x: 20
            y: parent.height / 2 - height / 2 - 70
            color: "black"
            radius: 30

            Rectangle {
                width: 115
                height: 115
                anchors.centerIn: parent
                color: (carinfo.throttle < 0) ? "#FF6868" : "white"
                radius: 22

                Text {
                    text: "R"
                    font.family: font.name
                    font.pixelSize: 130
                    color: (carinfo.throttle < 0) ? "white" : "#FF6868"
                    x: 13
                    y: -20
                }
            }
        }

        Rectangle {  // N
            width: 130
            height: 130
            x: 20
            y: parent.height / 2 - height / 2 + 70
            color: "black"
            radius: 30

            Rectangle {
                width: 115
                height: 115
                anchors.centerIn: parent
                color: (carinfo.throttle === 0) ? "#35CA3D" : "white"
                radius: 22

                Text {
                    text: "N"
                    font.family: font.name
                    font.pixelSize: 130
                    color: (carinfo.throttle === 0) ? "white" : "#35CA3D"
                    x: 13
                    y: -20
                }
            }
        }

        Rectangle {  // D
            width: 130
            height: 130
            x: 20
            y: parent.height / 2 - height / 2 + 210
            color: "black"
            radius: 30

            Rectangle {
                width: 115
                height: 115
                anchors.centerIn: parent
                color: (carinfo.throttle > 0) ? "#555555" : "white"
                radius: 22

                Text {
                    text: "D"
                    font.family: font.name
                    font.pixelSize: 130
                    color: (carinfo.throttle > 0) ? "white" : "#555555"
                    x: 13
                    y: -20
                }
            }
        }

        //==================================================//
        //                   Left Divider                   //
        //==================================================//

        Rectangle {
            width: 5
            height: parent.height
            anchors.verticalCenter: parent.verticalCenter
            x: parent.width / 2 - width / 2 - 340
            color: "black"
        }

        //==================================================//
        //                     Throttle                     //
        //==================================================//

        Text {
            text: Math.floor(carinfo.throttle * 100) + "%"
            font.family: font.name
            font.pixelSize: 70
            color: "black"
            x: 200
            y: 5
        }

        //==================================================//
        //                    Direction                     //
        //==================================================//

        Image {  // Left
            source: "../image/turn-icon.png"
            width: 100
            height: 100
            rotation: -90
            fillMode: Image.PreserveAspectFit
            opacity: valueSource.left_on_off ? 0.2 : 1.0
            x: parent.width / 2 - width / 2 - 170
            y: 485

            Image {
                source: "../image/turn-icon.png"
                width: 80
                height: 80
                fillMode: Image.PreserveAspectFit
                anchors.horizontalCenter: parent.horizontalCenter
                y: 12
            }
        }

        Image {  // Right
            source: "../image/turn-icon.png"
            width: 100
            height: 100
            rotation: 90
            fillMode: Image.PreserveAspectFit
            opacity: valueSource.right_on_off ? 0.2 : 1.0
            x: parent.width / 2 - width / 2 + 170
            y: 485

            Image {
                source: "../image/turn-icon.png"
                width: 80
                height: 80
                fillMode: Image.PreserveAspectFit
                anchors.horizontalCenter: parent.horizontalCenter
                y: 12
            }
        }

        //==================================================//
        //                    Emergency                     //
        //==================================================//

        Image {
            source: "../image/warning-icon.png"
            opacity: valueSource.emergency_on_off ? 0.3 : 1.0
            width: 120
            height: 120
            fillMode: Image.PreserveAspectFit
            anchors.horizontalCenter: parent.horizontalCenter
            y: 475

            Image {
                source: "../image/warning-icon.png"
                width: 90
                height: 90
                fillMode: Image.PreserveAspectFit
                anchors.horizontalCenter: parent.horizontalCenter
                y: 20

                Image {
                    source: "../image/warning-icon.png"
                    width: 65
                    height: 65
                    fillMode: Image.PreserveAspectFit
                    anchors.horizontalCenter: parent.horizontalCenter
                    y: 17
                }
            }

            MouseArea {
                anchors.fill: parent
                onClicked: {
                    valueSource.emergency = !valueSource.emergency
                }
            }
        }

        //==================================================//
        //                  Right Divider                   //
        //==================================================//

        Rectangle {
            width: 5
            height: parent.height
            anchors.verticalCenter: parent.verticalCenter
            x: parent.width / 2 - width / 2 + 340
            color: "black"
        }

        //==================================================//
        //                       Logo                       //
        //==================================================//

        Image {
            source: "../image/logo.png"
            width: 320
            height: 320
            fillMode: Image.PreserveAspectFit
            anchors.horizontalCenter: parent.horizontalCenter
            y: 80
        }

        Image {
            source: "../image/seame.png"
            width: 240
            height: 240
            fillMode: Image.PreserveAspectFit
            anchors.horizontalCenter: parent.horizontalCenter
            y: 315
        }

        Image {
            source: "../image/wolfsberg.png"
            width: 220
            height: 220
            fillMode: Image.PreserveAspectFit
            x: 830
            y: -40
        }

        //==================================================//
        //                      Clock                       //
        //==================================================//

        Item {
            width: 230
            height: 70
            x: 605
            y: 5

            Text {
                text: valueSource.clock
                font.family: font.name
                font.pixelSize: 70
                color: "black"
                anchors.right: parent.right
                y: 0
            }
        }

        //==================================================//
        //                  Ambient Light                   //
        //==================================================//

        Slider {  // Red
            id: redSlider
            width: 140
            x: 870
            y: 130
            minimumValue: 0
            maximumValue: 128
            stepSize: 16
            value: valueSource.red

            style: SliderStyle {
                groove: Rectangle {
                    implicitWidth: 140
                    implicitHeight: 8
                    color: "#FFCECE"
                    radius: 30
                }
                handle: Rectangle {
                    implicitWidth: 23
                    implicitHeight: 23
                    color: "#FF6868"
                    radius: 30
                }
            }

            onValueChanged: {
                valueSource.red_string = redSlider.value.toString(16)
                if (valueSource.red_string.length === 1) {
                    valueSource.red_string = "0" + valueSource.red_string
                }

                valueSource.light = "#" + valueSource.red_string + valueSource.green_string + valueSource.blue_string
            }
        }

        Slider {  // Green
            id: greenSlider
            width: 140
            x: 870
            y: 160
            minimumValue: 0
            maximumValue: 128
            stepSize: 16
            value: valueSource.green

            style: SliderStyle {
                groove: Rectangle {
                    implicitWidth: 140
                    implicitHeight: 8
                    color: "#AEFFAE"
                    radius: 30
                }
                handle: Rectangle {
                    implicitWidth: 23
                    implicitHeight: 23
                    color: "#35CA3D"
                    radius: 30
                }
            }

            onValueChanged: {
                valueSource.green_string = greenSlider.value.toString(16)
                if (valueSource.green_string.length === 1) {
                    valueSource.green_string = "0" + valueSource.green_string
                }

                valueSource.light = "#" + valueSource.red_string + valueSource.green_string + valueSource.blue_string
            }
        }

        Slider {  // Blue
            id: blueSlider
            width: 140
            x: 870
            y: 190
            minimumValue: 0
            maximumValue: 128
            stepSize: 16
            value: valueSource.blue

            style: SliderStyle {
                groove: Rectangle {
                    implicitWidth: 140
                    implicitHeight: 8
                    color: "#B1CAFF"
                    radius: 30
                }
                handle: Rectangle {
                    implicitWidth: 23
                    implicitHeight: 23
                    color: "#4D86FF"
                    radius: 30
                }
            }

            onValueChanged: {
                valueSource.blue_string = blueSlider.value.toString(16)
                if (valueSource.blue_string.length === 1) {
                    valueSource.blue_string = "0" + valueSource.blue_string
                }

                valueSource.light = "#" + valueSource.red_string + valueSource.green_string + valueSource.blue_string
            }
        }

        //==================================================//
        //                     GPS Mode                     //
        //==================================================//

        Rectangle {
            width: 140
            height: 60
            x: 870
            y: 240
            color: "black"
            radius: 20

            Rectangle {
                width: 130
                height: 50
                anchors.centerIn: parent
                color: valueSource.gps ? "#555555" : "white"
                radius: 15

                Text {
                    text: "GPS"
                    font.family: font.name
                    font.pixelSize: 40
                    color: valueSource.gps ? "white" : "555555"
                    x: 17
                    y: 1
                }
            }

            MouseArea {
                anchors.fill: parent
                onClicked: {
                    valueSource.gps = !valueSource.gps
                }
            }
        }

        Rectangle {
            visible: valueSource.gps
            width: 675
            height: 395
            color: "black"
            anchors.horizontalCenter: parent.horizontalCenter
            y: 75

            Image {
                source: "../image/map.png"
                width: 675
                height: 395
                fillMode: Image.PreserveAspectFit
                anchors.centerIn: parent
            }

            Rectangle {
                width: 30
                height: 30
                color: "white"
                radius: 15
                opacity: 0.7

                x: (Math.floor(carinfo.xposition * 78.5 + 51) > 675 - 15) ? 675 - 15 : ((Math.floor(carinfo.xposition * 78.5 + 51) < 0 - 15) ? 0 - 15 : Math.floor(carinfo.xposition * 78.5 + 51))
                y: (Math.floor(361 - carinfo.yposition * 78.5) < 0 - 15) ? 0 - 15 : ((Math.floor(361 - carinfo.yposition * 78.5) > 395 - 15) ? 395 - 15 : Math.floor(361 - carinfo.yposition * 78.5))
                rotation: Math.floor(carinfo.orientation * -1)

                Image {
                    source: "../image/location.png"
                    width: 30
                    height: 30
                    fillMode: Image.PreserveAspectFit
                    anchors.centerIn: parent
                }
            }
        }

        //==================================================//
        //                     ADS Mode                     //
        //==================================================//

        Rectangle {
            visible: true  // OTA UPDATE! [false -> true]

            width: 140
            height: 60
            x: 870
            y: 310
            color: "black"
            radius: 20

            Rectangle {
                width: 130
                height: 50
                anchors.centerIn: parent
                color: valueSource.ads ? "#FF6868" : "white"
                radius: 15

                Text {
                    text: "ADS"
                    font.family: font.name
                    font.pixelSize: 40
                    color: valueSource.ads ? "white" : "#FF6868"
                    x: 16
                    y: 1
                }
            }

            MouseArea {
                anchors.fill: parent
                onClicked: {
                    valueSource.ads = !valueSource.ads
                }
            }
        }

        //==================================================//
        //                   Power Button                   //
        //==================================================//

        Image {
            source: "../image/off.png"
            width: 100
            height: 100
            fillMode: Image.PreserveAspectFit
            opacity: 0.5
            x: 888
            y: 470
        }
    }
}
