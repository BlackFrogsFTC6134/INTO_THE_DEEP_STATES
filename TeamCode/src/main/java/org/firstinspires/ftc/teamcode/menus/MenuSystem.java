package org.firstinspires.ftc.teamcode.menus;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import java.util.ArrayList;
import java.util.List;

@TeleOp(name="Menu System", group="Test")
public class MenuSystem extends LinearOpMode {

    private List<MenuItem> mainMenu;
    private int currentMainIndex = 0;
    private int currentChildIndex = 0;
    private int currentGrandchildIndex = 0;
    private MenuState menuState = MenuState.MAIN;

    private enum MenuState {
        MAIN, CHILD, GRANDCHILD
    }

    @Override
    public void runOpMode() {
        initializeMenu();

        waitForStart();

        while (opModeIsActive()) {
            handleInput();
            displayMenu();
            telemetry.update();
        }
    }

    private void initializeMenu() {
        mainMenu = new ArrayList<>();
        for (int i = 1; i <= 5; i++) {
            MenuItem mainItem = new MenuItem("Main " + i);
            for (int j = 1; j <= 3; j++) {
                MenuItem childItem = new MenuItem("Child " + i + "-" + j);
                for (int k = 1; k <= 1; k++) {
                    childItem.addChild(new MenuItem("Grandchild " + i + "-" + j + "-" + k));
                }
                mainItem.addChild(childItem);
            }
            mainMenu.add(mainItem);
        }
    }

    private void handleInput() {
        if (gamepad1.dpad_up && !gamepad1.dpad_up) {
            navigateUp();
        } else if (gamepad1.dpad_down && !gamepad1.dpad_down) {
            navigateDown();
        } else if (gamepad1.dpad_right && !gamepad1.dpad_right) {
            navigateRight();
        } else if (gamepad1.dpad_left && !gamepad1.dpad_left) {
            navigateLeft();
        }
    }

    private void navigateUp() {
        switch (menuState) {
            case MAIN:
                currentMainIndex = (currentMainIndex - 1 + mainMenu.size()) % mainMenu.size();
                break;
            case CHILD:
                currentChildIndex = (currentChildIndex - 1 + mainMenu.get(currentMainIndex).getChildren().size()) % mainMenu.get(currentMainIndex).getChildren().size();
                break;
            case GRANDCHILD:
                currentGrandchildIndex = (currentGrandchildIndex - 1 + mainMenu.get(currentMainIndex).getChildren().get(currentChildIndex).getChildren().size()) % mainMenu.get(currentMainIndex).getChildren().get(currentChildIndex).getChildren().size();
                break;
        }
    }

    private void navigateDown() {
        switch (menuState) {
            case MAIN:
                currentMainIndex = (currentMainIndex + 1) % mainMenu.size();
                break;
            case CHILD:
                currentChildIndex = (currentChildIndex + 1) % mainMenu.get(currentMainIndex).getChildren().size();
                break;
            case GRANDCHILD:
                currentGrandchildIndex = (currentGrandchildIndex + 1) % mainMenu.get(currentMainIndex).getChildren().get(currentChildIndex).getChildren().size();
                break;
        }
    }

    private void navigateRight() {
        switch (menuState) {
            case MAIN:
                menuState = MenuState.CHILD;
                currentChildIndex = 0;
                break;
            case CHILD:
                menuState = MenuState.GRANDCHILD;
                currentGrandchildIndex = 0;
                break;
        }
    }

    private void navigateLeft() {
        switch (menuState) {
            case CHILD:
                menuState = MenuState.MAIN;
                break;
            case GRANDCHILD:
                menuState = MenuState.CHILD;
                break;
        }
    }

    private void displayMenu() {
        telemetry.addData("Use D-pad to navigate", "");
        telemetry.addData("Current Menu", menuState);
        telemetry.addLine();

        for (int i = 0; i < mainMenu.size(); i++) {
            String prefix = (i == currentMainIndex && menuState == MenuState.MAIN) ? ">>" : ">";
            telemetry.addData(prefix + " Main " + (i + 1), mainMenu.get(i).getName());

            if (i == currentMainIndex && (menuState == MenuState.CHILD || menuState == MenuState.GRANDCHILD)) {
                for (int j = 0; j < mainMenu.get(i).getChildren().size(); j++) {
                    String childPrefix = (j == currentChildIndex && menuState == MenuState.CHILD) ? "  >>" : "  >";
                    telemetry.addData(childPrefix + " Child " + (i + 1) + "-" + (j + 1), mainMenu.get(i).getChildren().get(j).getName());

                    if (j == currentChildIndex && menuState == MenuState.GRANDCHILD) {
                        for (int k = 0; k < mainMenu.get(i).getChildren().get(j).getChildren().size(); k++) {
                            String grandchildPrefix = (k == currentGrandchildIndex) ? "    >>" : "    >";
                            telemetry.addData(grandchildPrefix + " Grandchild " + (i + 1) + "-" + (j + 1) + "-" + (k + 1),
                                    mainMenu.get(i).getChildren().get(j).getChildren().get(k).getName());
                        }
                    }
                }
            }
        }
    }

    private class MenuItem {
        private String name;
        private List<MenuItem> children;

        public MenuItem(String name) {
            this.name = name;
            this.children = new ArrayList<>();
        }

        public void addChild(MenuItem child) {
            children.add(child);
        }

        public String getName() {
            return name;
        }

        public List<MenuItem> getChildren() {
            return children;
        }
    }
}