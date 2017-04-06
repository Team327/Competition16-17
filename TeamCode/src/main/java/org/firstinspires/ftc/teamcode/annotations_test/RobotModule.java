package org.firstinspires.ftc.teamcode.annotations_test;

import com.google.common.base.Function;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.Map;

/**
 * Created by roboticsteam on 4/5/2017.
 */

@Module(name="")
public abstract class RobotModule {
    private RobotCore core = RobotCore.getInstance();

    private Map<String, RobotModule> dependencies;
    private Map<String, HardwareDevice> devices;
    private Map<String, Method> methods;

    protected RobotModule() {
        dependencies = core.resolveDependencies(this.getClass().getAnnotation(Module.class).dependents());
        devices = null; //TODO do this
        methods = null; //TODO do this
    }

    //TODO Problem: Somehow handle circular dependencies

    public <Any> Any call(String methodName, Object... args) {
        return null; //TODO
    }

    protected <T extends RobotModule> T dependent(String typeName) {
        return null; //TODO
    }

    //TODO don't make function generic (except in this case it may be appropriate)
    private Function generateFunction(Method method, Class<?> paramTypes) {
        Class<?> returnType = method.getReturnType();
//        Function funct = this::call;
        return null; //TODO
    }

    private <T> T call(Method method, Object... params) {
        try {
            return (T) method.invoke(this, params);
        } catch(InvocationTargetException|IllegalAccessException e) {
            throw new IllegalArgumentException("Invalid use of method " + method.getName());
        }
    }

    private Method getMethod(String methodName, Class<?> paramTypes) {
        try {
            Method method;
            if(paramTypes == null) {
                method = this.getClass().getMethod(methodName);
            } else {
                method = this.getClass().getDeclaredMethod(methodName, paramTypes);
            }

            if(!method.isAnnotationPresent(Hidden.class)) {
                return method;
            } else {
                throw new IllegalArgumentException("@Hidden tag on method " + methodName);
            }
        } catch (NoSuchMethodException e) {
            throw new IllegalArgumentException("No method " + methodName);
        }
        //TODO make functions to call methods by type in subclass
    }
}
