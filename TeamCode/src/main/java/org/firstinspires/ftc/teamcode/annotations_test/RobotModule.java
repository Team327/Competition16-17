package org.firstinspires.ftc.teamcode.annotations_test;

import java.lang.reflect.Method;

/**
 * Created by roboticsteam on 4/5/2017.
 */

@Module(name="")
public abstract class RobotModule {
    private RobotCore core = RobotCore.getInstance();
    private RobotAnnotations annotations = RobotAnnotations.getInstance();

    private RobotModule[] dependencies;

    protected RobotModule() {
        dependencies = core.resolveDependencies(this.getClass().getAnnotation(Module.class).dependents());
    }

    //TODO Problem: Somehow handle circular dependencies

    public void getMethod(String methodName, Class<?> paramTypes) {
        try {
            if(paramTypes == null) {
                Method method = this.getClass().getMethod(methodName);
            } else {
                Method method = this.getClass().getDeclaredMethod(methodName, paramTypes);
            }
        } catch (NoSuchMethodException e) {
            throw new IllegalArgumentException("No method " + methodName);
        }
        //TODO make functions to call methods by type in subclass
    }
}
