import faq_db  # your database helper module

def main():
    faq_db.init_db()  # initialize database
    print("💡 ECEN FAQ Terminal Interface")
    
    while True:
        print("\nSelect an option:")
        print("1. Add a new FAQ")
        print("2. View all FAQs")
        print("3. Search FAQs")
        print("4. Exit")
        
        choice = input("Enter your choice (1-4): ").strip()
        
        if choice == "1":
            course_code = input("Enter course code (e.g., ECEN 325): ").strip()
            question = input("Enter question: ").strip()
            answer = input("Enter answer: ").strip()
            if course_code and question and answer:
                faq_db.add_faq(course_code, question, answer)
            else:
                print("⚠️  All fields are required!")
        
        elif choice == "2":
            faqs = faq_db.get_all_faqs()
            if faqs:
                for f in faqs:
                    print(f"\nID: {f[0]}\nCourse: {f[1]}\nQ: {f[2]}\nA: {f[3]}\n{'-'*30}")
            else:
                print("No FAQs found.")
        
        elif choice == "3":
            keyword = input("Enter keyword to search: ").strip()
            results = faq_db.search_faqs(keyword)
            if results:
                for r in results:
                    print(f"\nID: {r[0]}\nCourse: {r[1]}\nQ: {r[2]}\nA: {r[3]}\n{'-'*30}")
            else:
                print("No matching FAQs found.")
        
        elif choice == "4":
            print("Exiting. Goodbye!")
            break
        else:
            print("Invalid choice. Please enter 1-4.")

if __name__ == "__main__":
    main()
