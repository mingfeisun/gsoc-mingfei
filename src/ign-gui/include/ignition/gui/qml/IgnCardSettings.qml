import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3
import QtQuick.Window 2.2
import "qrc:/qml"

Dialog {
  id: settingsDialog

  Column {
    id: settingsColumn
    anchors.horizontalCenter: settingsDialog.horizontalCenter
    width: settingsDialog.width * 0.6

    Switch {
      id: titleSwitch
      text: "Show title bar"
      checked: card.showTitleBar
      onToggled: {
        card.showTitleBar = checked
        // why is binding not working?
        closeSwitch.enabled = checked
        dockSwitch.enabled = checked
      }
    }

    Switch {
      id: closeSwitch
      text: "Show close button"
      visible: !card.standalone
      enabled: card.showTitleBar
      checked: card.showCloseButton
      onToggled: {
        card.showCloseButton = checked
      }
    }

    Switch {
      id: dockSwitch
      text: "Show dock button"
      visible: !card.standalone
      enabled: card.showTitleBar
      checked: card.showDockButton
      onToggled: {
        card.showDockButton = checked
      }
    }

    Switch {
      id: resizableSwitch
      text: "Resizable"
      visible: card.state === "floating"
      checked: card.resizable
      onToggled: {
        card.resizable = checked
      }
    }

    GridLayout {
      width: parent.width
      columns: 2
      visible: card.state === "floating"

      Label {
        text: "Position"
        font.weight: Font.DemiBold
      }

      Text {
        text: ""
      }

      // TODO(louise) Support setting anchors from the dialog
      Button {
        visible: card.anchored
        text: "Clear anchors"
        Layout.columnSpan: 2
        onClicked: {
          card.clearAnchors()
        }
      }

      IgnSpinBox {
        visible: !card.anchored
        maximumValue: card.parent ? card.parent.width - card.width : minSize
        onVisibleChanged: value = card.x
        onValueChanged: {
          card.x = value;
        }
      }
      Label {
        visible: !card.anchored
        text: "X"
      }
      IgnSpinBox {
        visible: !card.anchored
        maximumValue: card.parent ? card.parent.height - card.height : minSize
        onVisibleChanged: value = card.y
        onValueChanged: {
          card.y = value;
        }
      }
      Label {
        visible: !card.anchored
        text: "Y"
      }
      IgnSpinBox {
        visible: !card.anchored
        maximumValue: 10000
        onVisibleChanged: value = card.z
        onValueChanged: {
          card.z = value;
        }
      }
      Label {
        visible: !card.anchored
        text: "Z"
      }
      Label {
        text: "Size"
        font.weight: Font.DemiBold
      }
      Text {
        text: ""
      }
      IgnSpinBox {
        maximumValue: card.parent ? card.parent.width : minSize
        onVisibleChanged: {
          if (card)
            value = card.width
        }
        onValueChanged: {
          card.width = value;
        }
      }
      Label {
        text: "Width"
      }
      IgnSpinBox {
        maximumValue: card.parent ? card.parent.height : minSize
        onVisibleChanged: {
          if (card)
            value = card.height
        }
        onValueChanged: {
          card.height = value;
        }
      }
      Label {
        text: "Height"
      }
    }
  }
}
