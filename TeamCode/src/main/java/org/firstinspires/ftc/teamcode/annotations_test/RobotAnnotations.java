package org.firstinspires.ftc.teamcode.annotations_test;


import org.reflections.Reflections;

import java.lang.reflect.InvocationTargetException;
import java.util.Map;
import java.util.Set;
import java.util.TreeMap;

/**
 * Created by roboticsteam on 4/5/2017.
 */

public class RobotAnnotations {
    private static RobotAnnotations ourInstance = new RobotAnnotations();

    private static Reflections reflections = new Reflections("org.firstinspires.ftc.teamcode");

    public static RobotAnnotations getInstance() {
        return ourInstance;
    }

    private Map<String, Class<? extends RobotModule>> moduleTypes;

    private RobotAnnotations() {
        moduleTypes = getModuleTypes(); //get all modules
    }

    /*
     * TODO (non-static) methods to provide the usage we want
     * * Given name, initialize an instance of the associated type
     */

    public RobotModule create(String typeName) {
        return create(typeName, null);
    }

    //Creates an instance of the type associated with the name
    public RobotModule create(String typeName, Object[] args) {
        Class<? extends RobotModule> type = getType(typeName); //throws InvalidArgument if not valid
        try {
            if(args == null) {
                //no arguments to constructor
                return type.newInstance(); //creates instance of extending type with default constructor
            } else {
                //there are args to pass
                Class<?>[] argTypes = new Class<?>[args.length];
                for(int i = 0; i < args.length; i++) {
                    argTypes[i] = args[i].getClass();
                }
                try {
                    //Call constructor created by types of arguments
                    //TODO make sure it doesn't mess up with extended types
                    //TODO possibly loop through all constructors and test if they are valid subclass args
                    return RobotModule.class.getDeclaredConstructor(argTypes).newInstance(args);
                } catch(NoSuchMethodException|SecurityException|InvocationTargetException e) {
                    throw new IllegalArgumentException("Invalid constructor for module " + typeName);
                }
            }
        } catch(IllegalAccessException|InstantiationException e) {
            throw new IllegalArgumentException("Exception thrown from module " + typeName);
        }
    }

    public Class<? extends RobotModule> getType(String typeName) {
        if(moduleTypes.containsKey(typeName)) {
            return moduleTypes.get(typeName);
        } else {
            throw new IllegalArgumentException("No module type " + typeName);
        }
    }

    private Map<String, Class<? extends RobotModule>> getModuleTypes() {
        Map<String, Class<? extends RobotModule>> modules = new TreeMap<>();

        Set<Class<? extends RobotModule>> moduleTypes = reflections.getSubTypesOf(RobotModule.class);

        //Loop over each Class which has annotation
        for (Class<? extends RobotModule> moduleType : moduleTypes) {
            //Determine if the class is a valid RobotModule
            if (moduleType.isAnnotationPresent(Module.class)) {
                modules.put(moduleType.getAnnotation(Module.class).name(), moduleType);
            }
        }
        return modules;
    }
}
