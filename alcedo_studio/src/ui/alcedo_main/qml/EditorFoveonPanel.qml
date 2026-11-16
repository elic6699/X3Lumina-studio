import QtQuick
import QtQuick.Controls
import QtQuick.Dialogs
import QtQuick.Layouts
import Alcedo.Main 1.0

// Foveon native chain (path B). This panel does NOT go through
// EditorParameterCatalog / DevelopPayload: path B is a display-referred CPU
// sidecar producing a 16-bit sRGB finished image, so its state lives in
// FoveonSessionController and a per-file .x3fedit.json sidecar. Path A (the
// default wide-gamut DirectRgb Develop DAG) is left completely untouched.
Item {
    id: root
    objectName: "editorAdjustmentPanel_foveon"

    property var theme: null
    property var editorSession: null
    property bool controlsEnabled: true
    property var foveon: appModules ? appModules.foveonSession : null

    readonly property color colText: appTheme.textColor
    readonly property color colMuted: appTheme.textMutedColor
    readonly property color colAccent: appTheme.accentColor
    readonly property color colCardSurface: appTheme.cardSurfaceColor
    readonly property color colCardBorder: appTheme.cardBorderColor

    property var specs: []
    property var values: ({})
    // X3Lumina's SPP mode keeps exactly these six controls; they are applied
    // by SPP's own camera-code-value tone stage, not by the Alcedo DAG.
    readonly property var sppParameterKeys: [
        "exposure", "contrast", "shadow", "highlight", "saturation", "nr_luma"
    ]

    // Only the controls that have no Alcedo equivalent live here. Exposure,
    // contrast, colour, HSL, curves, WB, monochrome and split toning are the
    // main application's job (its existing Develop panel), so repeating them
    // here would be a second, divergent tone pass.
    readonly property var groups: [
        // Fill light is a Tone slider now (under Blacks), for every document.
        { key: "highlight", title: qsTr("Highlight") },
        { key: "detail", title: qsTr("Detail") },
        { key: "nr", title: qsTr("Noise Reduction") },
        { key: "nr_post", title: qsTr("Post Noise Reduction") },
        { key: "spp", title: qsTr("SPP Tone (原链解码)") }
    ]

    function refreshSpecs() {
        if (!root.foveon)
            return
        try { root.specs = JSON.parse(root.foveon.parametersJson) }
        catch (e) { root.specs = [] }
        root.refreshValues()
    }

    function refreshValues() {
        if (!root.foveon)
            return
        try { root.values = JSON.parse(root.foveon.valuesJson) }
        catch (e) { root.values = ({}) }
    }

    // Views.swift:3444-3498. Model gate: Quattro -> mode8 / hole-fix;
    // Merrill + classic -> sky neutralisation; SDQ highlight is Quattro-only
    // (the C chain ignores it on other bodies). "只作用于黄/橙" only appears
    // when 亮部保色 > 1.
    //
    // Gating uses modelGen, NOT modelClass: SD Quattro / SD Quattro H have
    // their own RAW type_format (SDQ 0x25 / SDQH 0x27), so the narrow Quattro
    // array probe would grey out their oe selector and highlight controls.
    // "SDQ 高光" is gated by the DigitalISOGain probe instead, which is exactly
    // the set of bodies where the normalisation has any effect.
    function gateOk(spec) {
        if (!root.foveon || !root.foveon.available)
            return false
        const gen = root.foveon.modelGen
        if (spec.gate === "quattro")
            return gen === 2
        if (spec.gate === "nonquattro")
            return gen >= 0 && gen !== 2
        if (spec.gate === "sdq")
            return root.foveon.sdqHlCapable
        if (spec.gate === "hl_chroma")
            return Number(root.values["hl_chroma"] || 1) > 1.0
        return true
    }

    function groupSpecs(group) {
        // 原链解码 yields a finished SPP image. X3Lumina keeps only the six
        // SPP tone controls; all other path-B controls would be a second tone
        // pass. These six are applied inside SPP's tone stage by the C bridge.
        if (root.foveon && root.foveon.sppMode) {
            if (group !== "spp")
                return []
            return root.specs.filter(function (s) {
                return root.sppParameterKeys.indexOf(s.key) >= 0
            })
        }
        if (group === "spp")
            return []
        // Fill light moved to the Tone panel (under Blacks) so every document can
        // use it; the decode copy is forced to zero, so keeping the slider here
        // would be a second, inert control.
        return root.specs.filter(function (s) {
            return s.group === group && s.key !== "fill_light"
        })
    }

    Component.onCompleted: root.refreshSpecs()

    FileDialog {
        id: foveonExportDialog
        title: qsTr("Export 16-bit sRGB (Foveon / SPP)")
        fileMode: FileDialog.SaveFile
        defaultSuffix: "tif"
        nameFilters: [
            qsTr("16-bit TIFF (*.tif *.tiff)"),
            qsTr("16-bit PNG (*.png)")
        ]
        onAccepted: if (root.foveon)
                        root.foveon.ExportFoveon16(selectedFile)
    }

    // The controller is a context object that only exists once the editor UI is
    // up, so a plain Connections created with the panel binds a null target and
    // Qt then reports "no signal of the target matches the name" for every
    // handler — silently: values, body gating and the digest commit all stop
    // following the controller. Creating it through a Loader guarantees the
    // target is real when the connections are made.
    onFoveonChanged: root.refreshSpecs()

    Loader {
        active: root.foveon !== null && root.foveon !== undefined
        sourceComponent: Connections {
            target: root.foveon
            function onParametersChanged() { root.refreshValues() }
            function onSourceChanged() { root.refreshSpecs() }
            function onDecoderChanged() { root.refreshSpecs() }
            // A decode-level operator is a document change: echo the new digest
            // through the RAW Decode node so the frame re-renders, enters
            // history and moves the result key.
            function onDecodeParamsChanged() { root.commitDecodeDigest(false) }
        }
    }

    // The Foveon operators live inside the decode, so the document has to be
    // told they moved or nothing re-renders. Field key is "raw_decode": "raw"
    // does not exist and submitPatch would reject the patch silently.
    function commitDecodeDigest(settled) {
        if (!root.foveon || !root.editorSession)
            return false
        if (typeof root.editorSession.submitPatch !== "function")
            return false
        return root.editorSession.submitPatch(
                    "raw_decode",
                    JSON.stringify({ foveon_digest: String(root.foveon.decodeDigest) }),
                    settled)
    }

    ColumnLayout {
        anchors.fill: parent
        spacing: 8

        // ---- A / B pathway switch -----------------------------------------
        Rectangle {
            Layout.fillWidth: true
            Layout.preferredHeight: pathColumn.implicitHeight + 20
            color: root.colCardSurface
            border.color: root.colCardBorder
            radius: 6
            ColumnLayout {
                id: pathColumn
                anchors.fill: parent
                anchors.margins: 10
                spacing: 6
                RowLayout {
                    Layout.fillWidth: true
                    Label {
                        text: qsTr("Decode backend")
                        color: root.colText
                        font.pixelSize: 11
                    }
                    ComboBox {
                        id: backendPicker
                        objectName: "foveonDecoderBackendPicker"
                        Layout.fillWidth: true
                        visible: root.foveon && root.foveon.decoderChoiceAvailable
                        model: [qsTr("Fuse解码"), qsTr("原链解码")]
                        currentIndex: root.foveon && root.foveon.sppMode ? 1 : 0
                        onActivated: if (root.foveon)
                                         root.foveon.SetDecoderBackend(index === 1 ? "spp" : "fuse")
                    }
                    // Quattro is fixed to own, classic to fuse; Merrill files
                    // without a usable SPP chain are fixed to fuse as well.
                    Label {
                        Layout.fillWidth: true
                        visible: !(root.foveon && root.foveon.decoderChoiceAvailable)
                        color: root.colMuted
                        font.pixelSize: 11
                        text: !root.foveon ? ""
                              : root.foveon.modelClass === 2 ? qsTr("own (fixed)")
                              : root.foveon.modelClass === 1 ? qsTr("fuse (fixed, no SPP chain)")
                                                             : qsTr("fuse (fixed)")
                    }
                }
                // No path A / path B switch: the decoder is fixed by the body.
                // Quattro X3Fs decode with own, classic bodies with fuse, and
                // only Merrill offers a choice (fuse decoding vs its SPP
                // native chain). Presenting A/B as a user option was wrong.
                Label {
                    Layout.fillWidth: true
                    wrapMode: Text.WordWrap
                    visible: root.foveon && root.foveon.sppMode
                    color: root.colAccent
                    font.pixelSize: 11
                    text: qsTr("Merrill 原链解码 is active. The SPP native chain outputs a finished 16-bit sRGB image. Exposure / contrast / shadows / highlights / saturation / luma NR are applied inside SPP's own tone stage; the other Foveon controls would be a second tone pass and are hidden. Changing luma NR rebuilds the prepared SPP buffer.")
                }
                Label {
                    Layout.fillWidth: true
                    wrapMode: Text.WordWrap
                    color: root.colMuted
                    font.pixelSize: 11
                    text: qsTr("Decoder: Quattro bodies (DP-Q, SD Quattro, SD Quattro H, SFD) always decode with own; classic bodies always with fuse. 原链解码 is not a third decoder — it is a second path inside Merrill's fuse decode and only exists there, so only Merrill shows the choice.")
                }
                Label {
                    Layout.fillWidth: true
                    wrapMode: Text.WordWrap
                    color: root.colMuted
                    font.pixelSize: 11
                    text: qsTr("These controls run inside the decode, before the Develop graph, so they are live in both fuse and own. Under 原链解码 the SPP chain owns the pixels and only its own six tone controls apply. Everything downstream — exposure, contrast, colour, curves, WB — is the main Develop panels' job and is deliberately not repeated here.")
                }
                Label {
                    Layout.fillWidth: true
                    elide: Text.ElideMiddle
                    color: root.colMuted
                    font.pixelSize: 11
                    text: root.foveon && root.foveon.available
                          ? root.foveon.sourcePath + "  ·  " + root.foveon.status
                          : qsTr("Open an X3F / X3I file to enable this panel.")
                }
            }
        }

        ColumnLayout {
            Layout.fillWidth: true
            spacing: 6
            RowLayout {
                Layout.fillWidth: true
                Button {
                    text: qsTr("Reset")
                    enabled: root.foveon && root.foveon.available
                    onClicked: {
                        root.foveon.ResetParams()
                        root.commitDecodeDigest(true)
                    }
                }
                Button {
                    text: qsTr("Load sidecar")
                    enabled: root.foveon && root.foveon.available
                    onClicked: {
                        root.foveon.LoadSidecar()
                        root.commitDecodeDigest(true)
                    }
                }
                Button {
                    text: qsTr("Save sidecar")
                    enabled: root.foveon && root.foveon.available
                    onClicked: root.foveon.SaveSidecar()
                }
            }
            Button {
                Layout.fillWidth: true
                text: qsTr("Export 16-bit sRGB (Foveon / SPP)")
                enabled: root.foveon && root.foveon.available
                         && root.controlsEnabled && !root.foveon.exporting
                onClicked: foveonExportDialog.open()
            }
        }

        // ---- Full adjustment panel ----------------------------------------
        ScrollView {
            Layout.fillWidth: true
            Layout.fillHeight: true
            clip: true
            contentWidth: availableWidth

            ColumnLayout {
                width: parent.width
                spacing: 10

                Repeater {
                    model: root.groups
                    delegate: ColumnLayout {
                        width: parent.width
                        spacing: 4
                        visible: root.groupSpecs(modelData.key).length > 0
                        Label {
                            text: modelData.title
                            color: root.colAccent
                            font.pixelSize: 12
                            font.bold: true
                        }
                        Label {
                            Layout.fillWidth: true
                            wrapMode: Text.WordWrap
                            color: root.colMuted
                            font.pixelSize: 10
                            visible: modelData.key === "highlight"
                            text: qsTr("Overexposure correction is a two-state switch (Off / mode8) — there is no intensity slider. Highlight de-pinking is Quattro-only; sky neutralisation applies to Merrill / classic. The three fixes are ON by default in this fork.")
                        }
                        Repeater {
                            model: root.groupSpecs(modelData.key)
                            delegate: RowLayout {
                                id: controlRow
                                width: parent.width
                                spacing: 6
                                readonly property bool gated: root.gateOk(modelData)
                                // The decode always runs through the native
                                // chain, so these are live whenever an X3F is
                                // open; 原链解码 renders its own product and
                                // hides the groups instead.
                                enabled: gated && root.controlsEnabled && root.foveon
                                         && root.foveon.available

                                Label {
                                    text: modelData.label
                                    color: controlRow.enabled ? root.colText : root.colMuted
                                    font.pixelSize: 11
                                    Layout.fillWidth: true
                                    elide: Text.ElideRight
                                }
                                Label {
                                    visible: modelData.kind === "scalar"
                                    color: root.colMuted
                                    font.pixelSize: 10
                                    text: Number(root.values[modelData.key] !== undefined
                                                 ? root.values[modelData.key] : modelData.minimum)
                                          .toFixed(modelData.decimals)
                                }
                                Slider {
                                    visible: modelData.kind === "scalar"
                                    Layout.preferredWidth: 130
                                    from: modelData.minimum
                                    to: modelData.maximum
                                    stepSize: modelData.step
                                    value: Number(root.values[modelData.key] !== undefined
                                                  ? root.values[modelData.key] : modelData.minimum)
                                    onMoved: if (root.foveon)
                                                 root.foveon.SetParam(modelData.key, value)
                                    // QQC2 Slider has no released() signal; the
                                    // press ending is the "settled" edge that
                                    // closes the history step.
                                    // QQC2 Slider has no released() signal; the
                                    // press ending is the "settled" edge that
                                    // closes the history step.
                                    onPressedChanged: if (!pressed)
                                                          root.commitDecodeDigest(true)
                                }
                                CheckBox {
                                    visible: modelData.kind === "toggle"
                                    checked: Number(root.values[modelData.key] || 0) >= 0.5
                                    onToggled: {
                                        if (root.foveon)
                                            root.foveon.SetParam(modelData.key, checked ? 1 : 0)
                                        root.commitDecodeDigest(true)
                                    }
                                }
                                ComboBox {
                                    visible: modelData.kind === "combo"
                                    Layout.preferredWidth: 130
                                    model: modelData.options
                                    currentIndex: Math.max(0, modelData.options.indexOf(
                                                               String(root.values[modelData.key] || "")))
                                    onActivated: {
                                        if (root.foveon)
                                            root.foveon.SetParam(modelData.key, currentText)
                                        root.commitDecodeDigest(true)
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}
