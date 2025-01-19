// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.Arrays;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.stream.Collectors;

import frc.robot.Constants;

public class Dashboard
{
    private static Dashboard _instance = null;

    public static Dashboard getInstance()
    {
        if (_instance == null)
        {
            _instance = new Dashboard();
        }

        return _instance;
    }

    /** Creates a new Dashboard. */
    private Dashboard()
    {
    }

    public void startHost()
    {
        ExecutorService executor = Executors.newSingleThreadExecutor();
        executor.submit(() -> startServer());
    }

    private static void startServer()
    {
        try (ServerSocket serverSocket = new ServerSocket(Constants.Dashboard.DASHBOARD_PORT))
        {
            serverSocket.setReuseAddress(true);

            System.out.println("Server is waiting for a connection...");

            while (true)
            {
                Socket clientSocket = serverSocket.accept();
                System.out.println("Client connected: " + clientSocket.getInetAddress());

                BufferedReader input  = new BufferedReader(new InputStreamReader(clientSocket.getInputStream()));
                PrintWriter    output = new PrintWriter(clientSocket.getOutputStream(), true);

                handleClientCommunication(input, output);

                clientSocket.close();
                System.out.println("Client disconnected. Waiting for a new connection");
            }
        }
        catch (IOException e)
        {
            e.printStackTrace();
        }
    }

    private static void handleClientCommunication(BufferedReader input, PrintWriter output)
    {
        try
        {
            String clientMessage;

            while ((clientMessage = input.readLine()) != null)
            {
                System.out.println("Received message: " + clientMessage);

                var messageParts   = clientMessage.split(":", 2);
                var messageType    = messageParts[0].toUpperCase();
                var messageContent = messageParts[1];

                String serverResponse;

                try
                {
                    serverResponse = switch (messageType)
                    {
                        case "QUERY" -> queryReply(messageContent);
                        case "GET" -> getReply(messageContent);
                        case "SET" -> setReply(messageContent);
                        case "EVENT" -> eventReply(messageContent);
                        case "BUTTON" -> buttonReply(messageContent);
                        case "PING" -> pingReply();
                        default -> "NACK";
                    };
                }
                catch (Exception e)
                {
                    e.printStackTrace();
                    serverResponse = "NACK";
                }

                output.println(serverResponse);
            }
        }
        catch (IOException e)
        {
            e.printStackTrace();
        }
        finally
        {
            try
            {
                output.close();
                input.close();
            }
            catch (IOException e)
            {
                e.printStackTrace();
            }
        }
    }

    private static String queryReply(String rawMessage)
    {
        return "";
    }

    private static String getReply(String rawMessage) throws Exception
    {
        return Arrays.stream(rawMessage.split(",")).mapToInt(Integer::parseInt).mapToObj(i ->
        {
            String response = switch (i)
            {
                case 0 -> "0|false";
                case 1 -> "1|hello";
                case 2 -> "2|3.14";
                case 3 -> "3|NACK";
                case 4 -> "4|7";
                default -> "";
            };

            return response;
        }).collect(Collectors.joining(","));
    }

    private static String setReply(String rawMessage)
    {
        return "";
    }

    private static String eventReply(String rawMessage)
    {
        return "EVENT:notice,event request received|error,robot collision detected";
    }

    private static String buttonReply(String rawMessage)
    {
        return "";
    }

    private static String pingReply()
    {
        return "PONG";
    }
}
