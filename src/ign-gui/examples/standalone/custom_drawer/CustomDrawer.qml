import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Controls.Material 2.1

/**
 * Custom drawer
 */
Rectangle {
  id: customDrawer
  anchors.fill: parent

  /**
   * Callback for list items
   */
  function onAction(_action) {
    switch(_action) {
      // Handle custom actions
      case "cppActionFromQml":
        CustomActions.cppActionFromQml()
        break
      // Forward others to default drawer
      default:
        parent.onAction(_action);
        break
    }
  }

  ListModel {
    id: drawerModel

    // Custom action which calls custom C++ code
    ListElement {
      title: "Call C++ action"
      action: "cppActionFromQml"
    }

    // Actions provided by Ignition GUI, with custom titles
    ListElement {
      title: "Call default action (Style)"
      action: "styleSettings"
    }

    ListElement {
      title: "Call default action (Quit)"
      action: "close"
    }
  }

  ListView {
    id: listView
    anchors.fill: parent

    delegate: ItemDelegate {
      // TODO(anyone): follow the application's style
      Material.theme: Material.theme
      width: parent.width
      text: title
      highlighted: ListView.isCurrentItem
      onClicked: {
        customDrawer.onAction(action);
        customDrawer.parent.closeDrawer();
      }
    }

    model: drawerModel

    ScrollIndicator.vertical: ScrollIndicator { }
  }
}
