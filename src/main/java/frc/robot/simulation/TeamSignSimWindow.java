package frc.robot.simulation;

import java.awt.Color;
import java.awt.Font;
import java.util.function.Supplier;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.SwingConstants;
import javax.swing.SwingUtilities;
import javax.swing.Timer;
import javax.swing.WindowConstants;

/**
 * Desktop-only pop-out that mimics the red LED team sign. Call {@link #open(Supplier)} from sim
 * (for example {@code Robot#simulationInit}) and pass a supplier that returns the latest formatted
 * line.
 */
public final class TeamSignSimWindow {

	private static final int kDefaultWindowWidthPx = 330;

	private static JFrame frame;
	private static Timer refreshTimer;

	private TeamSignSimWindow() {}

	/**
	 * Opens or focuses the window and refreshes the label from {@code lineSupplier} on the Swing
	 * timer (~10 Hz).
	 */
	public static void open(Supplier<String> lineSupplier) {
		SwingUtilities.invokeLater(
				() -> {
					if (frame == null) {
						frame = new JFrame("Team sign (sim)");
						frame.setDefaultCloseOperation(WindowConstants.HIDE_ON_CLOSE);
						JLabel label = new JLabel(" ", SwingConstants.CENTER);
						label.setOpaque(true);
						label.setBackground(Color.BLACK);
						label.setForeground(new Color(255, 40, 40));
						label.setFont(new Font(Font.MONOSPACED, Font.BOLD, 28));
						frame.add(label);
						frame.pack();
						frame.setSize(Math.max(kDefaultWindowWidthPx, frame.getWidth()), frame.getHeight());
						frame.setMinimumSize(frame.getSize());
						refreshTimer = new Timer(100, e -> {
							String text = lineSupplier.get();
							String next = text == null ? " " : text;
							if (!next.equals(label.getText())) {
								label.setText(next);
							}
						});
						refreshTimer.start();
					}
					frame.setVisible(true);
					frame.toFront();
				});
	} // End open

	/** Stops refresh and disposes the window if it was opened. */
	public static void close() {
		SwingUtilities.invokeLater(
				() -> {
					if (refreshTimer != null) {
						refreshTimer.stop();
						refreshTimer = null;
					}
					if (frame != null) {
						frame.dispose();
						frame = null;
					}
				});
	} // End close
}
