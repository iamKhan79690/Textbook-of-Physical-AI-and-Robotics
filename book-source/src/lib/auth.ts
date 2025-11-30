import { betterAuth } from "better-auth";
import Database from "better-sqlite3";
import path from "path";

// Create database instance
const dbPath = path.join(process.cwd(), "auth.db");
const db = new Database(dbPath);

export const auth = betterAuth({
    database: db,
    emailAndPassword: {
        enabled: true,
    },
    user: {
        additionalFields: {
            // Software background level
            softwareBackground: {
                type: "string",
                required: false,
                defaultValue: "beginner",
                input: true,
            },
            // Hardware experience level
            hardwareExperience: {
                type: "string",
                required: false,
                defaultValue: "none",
                input: true,
            },
            // Learning goals - free text
            learningGoals: {
                type: "string",
                required: false,
                defaultValue: "",
                input: true,
            },
            // Prior robotics experience
            hasRoboticsExperience: {
                type: "boolean",
                required: false,
                defaultValue: false,
                input: true,
            },
        },
    },
});
