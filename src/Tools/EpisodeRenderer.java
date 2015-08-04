package Tools;

import burlap.behavior.singleagent.EpisodeAnalysis;
import burlap.behavior.singleagent.EpisodeSequenceVisualizer;
import burlap.oomdp.auxiliary.StateParser;
import burlap.oomdp.core.Domain;
import burlap.oomdp.core.State;
import burlap.oomdp.visualizer.Visualizer;
import com.xuggle.mediatool.IMediaWriter;
import com.xuggle.mediatool.ToolFactory;

import javax.swing.*;
import javax.swing.event.ListSelectionEvent;
import javax.swing.event.ListSelectionListener;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.image.BufferedImage;
import java.util.List;
import java.util.concurrent.TimeUnit;

import static com.xuggle.xuggler.Global.DEFAULT_TIME_UNIT;

/**
 * @author James MacGlashan.
 */
public class EpisodeRenderer extends EpisodeSequenceVisualizer{


    protected int frameDelayMS = 16;
    protected int renderNumber = 0;

    public EpisodeRenderer(Visualizer v, Domain d, StateParser sp, String experimentDirectory) {
        super(v, d, sp, experimentDirectory);
    }

    public EpisodeRenderer(Visualizer v, Domain d, StateParser sp, String experimentDirectory, int w, int h) {
        super(v, d, sp, experimentDirectory, w, h);
    }

    public EpisodeRenderer(Visualizer v, Domain d, List<EpisodeAnalysis> episodes) {
        super(v, d, episodes);
    }

    public EpisodeRenderer(Visualizer v, Domain d, List<EpisodeAnalysis> episodes, int w, int h) {
        super(v, d, episodes, w, h);
    }

    public int getFrameDelayMS() {
        return frameDelayMS;
    }

    public void setFrameDelayMS(int frameDelayMS) {
        this.frameDelayMS = frameDelayMS;
    }


    /**
     * Initializes the GUI and presents it to the user.
     */
    @Override
    public void initGUI(){

        if(this.alreadyInitedGUI){
            return;
        }

        this.alreadyInitedGUI = true;

        //set viewer components
        propViewer = new TextArea();
        propViewer.setEditable(false);
        painter.setPreferredSize(new Dimension(cWidth, cHeight));
        propViewer.setPreferredSize(new Dimension(cWidth, 100));
        setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

        getContentPane().add(painter, BorderLayout.CENTER);
        getContentPane().add(propViewer, BorderLayout.SOUTH);



        //set episode component
        episodeList = new JList(episodesListModel);


        episodeList.setSelectionMode(ListSelectionModel.SINGLE_SELECTION);
        episodeList.setLayoutOrientation(JList.VERTICAL);
        episodeList.setVisibleRowCount(-1);
        episodeList.addListSelectionListener(new ListSelectionListener(){

            @Override
            public void valueChanged(ListSelectionEvent e) {
                handleEpisodeSelection(e);
            }
        });

        episodeScroller = new JScrollPane(episodeList);
        episodeScroller.setPreferredSize(new Dimension(100, 600));



        //set iteration component
        iterationListModel = new DefaultListModel();
        iterationList = new JList(iterationListModel);

        iterationList.setSelectionMode(ListSelectionModel.SINGLE_SELECTION);
        iterationList.setLayoutOrientation(JList.VERTICAL);
        iterationList.setVisibleRowCount(-1);
        iterationList.addListSelectionListener(new ListSelectionListener(){
            public void valueChanged(ListSelectionEvent e){
                handleIterationSelection(e);
            }
        });

        iterationScroller = new JScrollPane(iterationList);
        iterationScroller.setPreferredSize(new Dimension(150, 600));

        //add render movie button
        JButton renderButton = new JButton("Render Episode Movie");
        renderButton.addActionListener(new ActionListener() {

            @Override
            public void actionPerformed(ActionEvent arg0) {
                handleEpisodeRender();

            }
        });


        //add episode-iteration lists to window
        controlContainer = new Container();
        controlContainer.setLayout(new BorderLayout());


        controlContainer.add(episodeScroller, BorderLayout.WEST);
        controlContainer.add(iterationScroller, BorderLayout.EAST);
        controlContainer.add(renderButton, BorderLayout.SOUTH);


        getContentPane().add(controlContainer, BorderLayout.EAST);



        //display the window
        pack();
        setVisible(true);

    }


    protected void handleEpisodeRender(){


        if(this.curEA == null){
            return ;
        }
        if(this.curEA.maxTimeStep() == 0){
            return ;
        }

        System.out.println("Rendering " + (this.curEA.maxTimeStep()) + " frames...");

        int videoStreamIndex = 0;
        int videoStreamId = 0;
        long frameRate = DEFAULT_TIME_UNIT.convert(this.frameDelayMS, TimeUnit.MILLISECONDS);
        int width = this.painter.getWidth();
        int height = painter.getHeight();

        IMediaWriter writer = ToolFactory.makeWriter("renderedEpisode_" + this.renderNumber + ".avi");
        writer.addVideoStream(videoStreamIndex, videoStreamId, width, height);


        long nextFrameTime = 0;
        for(int i = 0; i < this.curEA.numTimeSteps(); i++){
            State s = this.curEA.getState(i);
            BufferedImage frame = this.renderFrame(s);
            writer.encodeVideo(videoStreamIndex, frame, nextFrameTime, DEFAULT_TIME_UNIT);
            nextFrameTime += frameRate;
        }

        //pad 5 frames to ensure everything is displayed
        for(int i = 0; i < 5; i++){
            BufferedImage frame = this.renderFrame(this.curEA.getState(this.curEA.maxTimeStep()));
            writer.encodeVideo(videoStreamIndex, frame, nextFrameTime, DEFAULT_TIME_UNIT);
            nextFrameTime += frameRate;
        }

        writer.close();

        System.out.println("Finished rendering.");
        this.renderNumber++;

    }

    protected BufferedImage renderFrame(State s){

        BufferedImage offscreen = new BufferedImage(this.painter.getWidth(), this.painter.getHeight(), BufferedImage.TYPE_3BYTE_BGR);
        Graphics2D go = offscreen.createGraphics();

        go.setColor(this.painter.getBgColor());
        go.fill(new Rectangle(this.painter.getWidth(), this.painter.getHeight()));


        this.painter.getStateRenderLayer().updateState(s);
        this.painter.getStateRenderLayer().render(go, this.painter.getWidth(), this.painter.getHeight());

        return offscreen;

    }

}

