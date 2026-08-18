package mchorse.bbs_mod.settings.ui;

import mchorse.bbs_mod.client.BBSRendering;
import mchorse.bbs_mod.l10n.L10n;
import mchorse.bbs_mod.settings.values.base.BaseValue;
import mchorse.bbs_mod.settings.values.core.ValueString;
import mchorse.bbs_mod.ui.UIKeys;
import mchorse.bbs_mod.ui.framework.elements.UIElement;
import mchorse.bbs_mod.ui.framework.elements.buttons.UIIcon;
import mchorse.bbs_mod.ui.framework.elements.input.text.UITextbox;
import mchorse.bbs_mod.ui.framework.elements.utils.UILabel;
import mchorse.bbs_mod.ui.utils.UIConstants;
import mchorse.bbs_mod.ui.utils.UIUtils;
import mchorse.bbs_mod.ui.utils.icons.Icons;

import java.util.Collections;
import java.util.List;

/**
 * The export path over the full width with a button that opens the folder it
 * points at — a path doesn't fit the narrow field the other settings use. Empty
 * means the default movies folder, so the button opens wherever the videos
 * actually land rather than what the field says.
 */
public class UIExportPathRow implements UISettingsLayout.IValueRow
{
    private final ValueString path;

    public UIExportPathRow(ValueString path)
    {
        this.path = path;
    }

    @Override
    public List<BaseValue> getValues()
    {
        return Collections.singletonList(this.path);
    }

    @Override
    public List<UIElement> create(UIElement ui)
    {
        UITextbox textbox = UIValueFactory.stringUI(this.path, null);
        UIIcon folder = new UIIcon(Icons.FOLDER, (b) -> UIUtils.openFolder(BBSRendering.getVideoFolder()));

        textbox.tooltip(L10n.lang(UIValueFactory.getValueCommentKey(this.path)));
        folder.tooltip(UIKeys.CAMERA_TOOLTIPS_OPEN_VIDEOS);
        /* As wide as the icon and as tall as the field beside it - a field is
         * CONTROL_HEIGHT tall, not the 20 its row slot gets */
        folder.wh(Icons.FOLDER.w, UIConstants.CONTROL_HEIGHT);

        UIElement row = new UIElement();

        row.row(4).height(UIConstants.CONTROL_HEIGHT);
        row.add(textbox, folder);

        UILabel label = UIValueFactory.label(this.path);

        label.h(10);

        /* The name belongs to the field under it, so they sit closer to each
         * other than to the settings around them */
        UIElement column = new UIElement();

        column.column(1).vertical().stretch();
        column.add(label, row);

        return Collections.singletonList(UIValueFactory.commetTooltip(column, this.path));
    }
}
