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
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

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
        try (ServerSocket serverSocket = new ServerSocket(Constants.DASHBOARD_PORT))
        {
            System.out.println("Server is waiting for a connection...");

            while (true)
            {
                Socket clientSocket = serverSocket.accept();
                System.out.println("Client connected: " + clientSocket.getInetAddress());

                BufferedReader input = new BufferedReader(new InputStreamReader(clientSocket.getInputStream()));
                PrintWriter output = new PrintWriter(clientSocket.getOutputStream(), true);

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
            int requestCount = 0;

            while ((clientMessage = input.readLine()) != null)
            {
                System.out.println("Received message: " + clientMessage);

                switch (clientMessage)
                {
                    case "PING":
                        output.println("PONG");
                        break;

                    case "DATA":
                        output.println(requestCount++);
                        break;

                    case "EVENT":
                        output.println("EVENT:notice,event request received|error,robot collision detected");
                        break;

                    default:
                        output.println("NACK");
                        break;
                }
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
}
