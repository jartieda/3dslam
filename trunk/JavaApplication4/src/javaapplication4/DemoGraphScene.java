/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package javaapplication4;


import java.awt.Image;
import java.awt.Point;
import java.util.Random;
import javax.swing.ImageIcon;
import org.netbeans.api.visual.action.ActionFactory;
import org.netbeans.api.visual.action.ConnectProvider;
import org.netbeans.api.visual.action.ConnectorState;
import org.netbeans.api.visual.anchor.AnchorFactory;
import org.netbeans.api.visual.anchor.AnchorShape;
import org.netbeans.api.visual.graph.GraphScene;
import org.netbeans.api.visual.widget.ConnectionWidget;
import org.netbeans.api.visual.widget.LayerWidget;
import org.netbeans.api.visual.widget.Scene;
import org.netbeans.api.visual.widget.Widget;
import org.netbeans.api.visual.widget.general.IconNodeWidget;
import org.openide.util.Utilities;

public class DemoGraphScene extends GraphScene {

    private LayerWidget mainLayer;
    private LayerWidget connectionLayer;
    private Image image;
    private Image bimage;
   
    public Scene scene;
    
    public DemoGraphScene() {
        
        mainLayer = new LayerWidget(this);
        addChild(mainLayer);
        connectionLayer = new LayerWidget(this);
        addChild(connectionLayer);
        scene=this;
        image = Utilities.icon2Image(new ImageIcon(getClass().getResource("target.png")));
        bimage = Utilities.icon2Image(new ImageIcon(getClass().getResource("play_icon.gif")));
        addNode("Chuk");
        addNode("Chuk2");
    }
    
    public void Add(){
        addNode("chuck");
        scene.validate();
    }
    
    @Override
    protected Widget attachNodeWidget(Object arg0) {
        
        
        IconNodeWidget widget = new IconNodeWidget(this);
        widget.setImage(image);
        widget.setPreferredLocation(new Point(10, 20));
        widget.getActions().addAction(ActionFactory.createExtendedConnectAction(connectionLayer, new SceneConnectProvider()));           
        widget.getActions().addAction(ActionFactory.createMoveAction());           
        mainLayer.addChild(widget);
        return widget;
    }

    @Override
    protected Widget attachEdgeWidget(Object arg0) {
        return null;
    }

    @Override
    protected void attachEdgeSourceAnchor(Object arg0, Object arg1, Object arg2) {
    }

    @Override
    protected void attachEdgeTargetAnchor(Object arg0, Object arg1, Object arg2) {
    }



class SceneConnectProvider implements ConnectProvider
{

    public boolean isSourceWidget(Widget arg0) {
        return true;
    }


    public boolean hasCustomTargetWidgetResolver(Scene arg0) {
        return false;
    }

    public Widget resolveTargetWidget(Scene arg0, Point arg1) {
        return null;
    }

    public void createConnection(Widget sourceWidget, Widget targetWidget) {
        ConnectionWidget connection = new ConnectionWidget(scene);
        connection.setTargetAnchorShape(AnchorShape.TRIANGLE_FILLED);
        connection.setSourceAnchor(AnchorFactory.createRectangularAnchor(sourceWidget));
        connection.setTargetAnchor(AnchorFactory.createRectangularAnchor(targetWidget));
        connectionLayer.addChild(connection);
    }

    public ConnectorState isTargetWidget(Widget sourceWidget, Widget targetWidget) {
        return sourceWidget != targetWidget && targetWidget instanceof IconNodeWidget ? ConnectorState.ACCEPT:ConnectorState.REJECT;
    }
   
}

}